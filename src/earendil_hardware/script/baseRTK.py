#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BASE SIDE

LC29HEA Base GPS -> 3DR SiK Telemetry Radio -> Rover

Task:
1. Puts Base LC29HEA into base mode.
2. Sets Survey-in or fixed ECEF base position.
3. Extracts RTCM3 packets coming from GPS.
4. Sends only valid RTCM3 frames to 3DR radio.
5. Prints short status messages coming from Rover to screen.

Note:
- MAVLINK=0 is recommended in 3DR SiK radio settings.
- NETID, baud and air speed must be the same on both radios.
"""

import serial
import threading
import time
from collections import Counter


# =======================================================
# 1) PORT AND BAUD SETTINGS
# =======================================================

BASE_GPS_PORT = "/dev/ttyUSB0"       # Windows example: COM8
BASE_RADIO_PORT = "/dev/ttyUSB1"    # Windows example: COM12

# Ubuntu example:
# BASE_GPS_PORT = "/dev/ttyUSB0"
# BASE_RADIO_PORT = "/dev/ttyUSB1"

GPS_BAUD = 460800
RADIO_BAUD = 57600

SERIAL_TIMEOUT = 0.05


# =======================================================
# 2) BASE CONFIGURATION
# =======================================================

CONFIGURE_BASE_ON_START = True

# "survey_in" or "fixed_ecef"
# If base antenna is placed at a different location on each setup, survey_in should be used.
BASE_POSITION_MODE = "survey_in"

# Survey-in setting
# In this version, the system averages the base position for 5 minutes on startup,
# then starts RTCM transmission threads.
SURVEY_IN_MIN_SEC = 400   # 5 minutes
SURVEY_IN_ACC_M = 0.5        # target survey-in accuracy: 0.5 m
WAIT_SURVEY_IN_BEFORE_RTCM = True
SURVEY_IN_EXTRA_WAIT_SEC = 10

# If you use Fixed ECEF, fill these.
# Values must be WGS84 ECEF X/Y/Z in meters.
FIXED_ECEF_X = 0.0
FIXED_ECEF_Y = 0.0
FIXED_ECEF_Z = 0.0

# Enabling RTCM messages
ENABLE_RTCM_1005 = True
ENABLE_RTCM_MSM = True

# If RF bandwidth is not enough, receiver's default MSM4 setting might be more stable than MSM7.
# For now, we are keeping the PAIR432 and PAIR434 logic from your old code.


# =======================================================
# 3) NMEA / QUECTEL COMMAND CHECKSUM
# =======================================================

def checksum(nmea_body: str) -> str:
    cs = 0
    for ch in nmea_body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def send_cmd(ser: serial.Serial, raw_cmd: str, wait: float = 0.25) -> None:
    """
    Sends Quectel/LC29H command with checksum.
    Example input:
        PQTMCFGRCVRMODE,W,2
    Sent:
        $PQTMCFGRCVRMODE,W,2*CS\\r\\n
    """
    body = raw_cmd.replace("$", "").split("*")[0]
    full_cmd = f"${body}*{checksum(body)}\r\n"

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(full_cmd.encode("ascii"))
    ser.flush()
    time.sleep(wait)

    try:
        resp = ser.read_all().decode("ascii", errors="ignore").strip()
        if resp:
            print(f"[GPS RESPONSE] {resp}")
        else:
            print(f"[NO GPS RESPONSE] {body}")
    except Exception as e:
        print(f"[GPS RESPONSE READ ERROR] {e}")


def configure_base_gps(gps_ser: serial.Serial) -> None:
    print("\n=== Base GPS configuration starting ===")

    # 1) Base station mode
    send_cmd(gps_ser, "PQTMCFGRCVRMODE,W,2")

    # 2) Base position
    if BASE_POSITION_MODE == "survey_in":
        cmd = f"PQTMCFGSVIN,W,1,{SURVEY_IN_MIN_SEC},{SURVEY_IN_ACC_M},0,0,0"
        send_cmd(gps_ser, cmd)
        print(f"[BASE MODE] Survey-in active: {SURVEY_IN_MIN_SEC}s, target accuracy {SURVEY_IN_ACC_M}m")

    elif BASE_POSITION_MODE == "fixed_ecef":
        if FIXED_ECEF_X == 0.0 and FIXED_ECEF_Y == 0.0 and FIXED_ECEF_Z == 0.0:
            print("[WARNING] fixed_ecef selected but ECEF values are 0.0. This is not correct.")
        cmd = f"PQTMCFGSVIN,W,2,0,0,{FIXED_ECEF_X},{FIXED_ECEF_Y},{FIXED_ECEF_Z}"
        send_cmd(gps_ser, cmd)
        print("[BASE MODE] Fixed ECEF active.")

    else:
        print("[WARNING] Invalid BASE_POSITION_MODE. A value other than Survey-in/fixed_ecef was entered.")

    # 3) Enable RTCM 1005
    if ENABLE_RTCM_1005:
        send_cmd(gps_ser, "PAIR434,1")

    # 4) Enable RTCM MSM
    if ENABLE_RTCM_MSM:
        send_cmd(gps_ser, "PAIR432,1")

    print("=== Base GPS configuration finished ===\n")


# =======================================================
# 4) RTCM3 PARSER
# =======================================================

def crc24q(data: bytes) -> int:
    """
    RTCM3 CRC24Q calculation.
    Polynomial: 0x1864CFB
    """
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc


def rtcm_message_type(frame: bytes) -> int:
    """
    RTCM message type is the first 12 bits.
    """
    payload = frame[3:-3]
    if len(payload) < 2:
        return -1
    return (payload[0] << 4) | (payload[1] >> 4)


def extract_rtcm3_frames(buffer: bytearray):
    """
    Extracts valid RTCM3 frames from buffer.
    RTCM3 frame:
        0xD3 | length 10-bit | payload | CRC24Q 3-byte
    """
    frames = []

    while True:
        start = buffer.find(b"\xD3")

        if start < 0:
            return bytearray(), frames

        if len(buffer) - start < 3:
            return bytearray(buffer[start:]), frames

        # Upper 6 bits of 2nd byte in RTCM header must be reserved = 0.
        if buffer[start + 1] & 0xFC:
            buffer = buffer[start + 1:]
            continue

        length = ((buffer[start + 1] & 0x03) << 8) | buffer[start + 2]
        total_len = 3 + length + 3

        if len(buffer) - start < total_len:
            return bytearray(buffer[start:]), frames

        frame = bytes(buffer[start:start + total_len])

        received_crc = (
            (frame[-3] << 16) |
            (frame[-2] << 8) |
            frame[-1]
        )
        calculated_crc = crc24q(frame[:-3])

        if received_crc == calculated_crc:
            frames.append(frame)
            buffer = buffer[start + total_len:]
        else:
            # If wrong D3 caught, advance by one byte.
            buffer = buffer[start + 1:]


# =======================================================
# 5) BASE GPS -> RADIO RTCM TRANSFER
# =======================================================

def gps_to_radio_rtcm(gps_ser: serial.Serial, radio_ser: serial.Serial, stop_event: threading.Event):
    print("[TRANSFER] Base GPS -> Radio RTCM started.")

    buffer = bytearray()
    last_report = time.time()
    byte_count = 0
    frame_count = 0
    msg_counter = Counter()

    while not stop_event.is_set():
        try:
            data = gps_ser.read(gps_ser.in_waiting or 1)

            if data:
                buffer.extend(data)
                buffer, frames = extract_rtcm3_frames(buffer)

                for frame in frames:
                    radio_ser.write(frame)
                    radio_ser.flush()

                    byte_count += len(frame)
                    frame_count += 1
                    msg_counter[rtcm_message_type(frame)] += 1

            now = time.time()
            if now - last_report >= 1.0:
                if frame_count > 0:
                    top_msgs = ", ".join(
                        f"{msg}:{cnt}" for msg, cnt in msg_counter.most_common(6)
                    )
                    print(f"[BASE TX] {byte_count} byte/s | {frame_count} frame/s | msg: {top_msgs}")
                else:
                    print("[BASE TX] No RTCM. Check if Base GPS is producing RTCM.")

                byte_count = 0
                frame_count = 0
                msg_counter.clear()
                last_report = now

        except Exception as e:
            print(f"[ERROR] GPS -> Radio RTCM transfer error: {e}")
            time.sleep(0.2)


# =======================================================
# 6) ROVER -> BASE STATUS MESSAGE READING
# =======================================================

def radio_to_console(radio_ser: serial.Serial, stop_event: threading.Event):
    print("[LISTENING] Rover status message listening started.")

    buffer = ""

    while not stop_event.is_set():
        try:
            data = radio_ser.read(radio_ser.in_waiting or 1)

            if not data:
                continue

            text = data.decode("ascii", errors="ignore")
            if not text:
                continue

            buffer += text

            # Prevent buffer bloat.
            if len(buffer) > 3000:
                buffer = buffer[-1500:]

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                if line.startswith("ROVER,"):
                    print(f"[ROVER STATUS] {line}")
                else:
                    print(f"[RADIO RX] {line}")

        except Exception as e:
            print(f"[ERROR] Radio console read error: {e}")
            time.sleep(0.2)


# =======================================================
# 7) MAIN
# =======================================================

def main():
    print("=== LC29HEA BASE -> 3DR SiK RTCM TRANSFER ===")

    try:
        gps_ser = serial.Serial(
            BASE_GPS_PORT,
            GPS_BAUD,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1
        )

        radio_ser = serial.Serial(
            BASE_RADIO_PORT,
            RADIO_BAUD,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1
        )

        gps_ser.reset_input_buffer()
        gps_ser.reset_output_buffer()
        radio_ser.reset_input_buffer()
        radio_ser.reset_output_buffer()

        print(f"[OK] Base GPS connected: {BASE_GPS_PORT} @ {GPS_BAUD}")
        print(f"[OK] Base Radio connected: {BASE_RADIO_PORT} @ {RADIO_BAUD}")

    except Exception as e:
        print(f"[CONNECTION ERROR] {e}")
        return

    if CONFIGURE_BASE_ON_START:
        configure_base_gps(gps_ser)

        if BASE_POSITION_MODE == "survey_in" and WAIT_SURVEY_IN_BEFORE_RTCM:
            wait_sec = SURVEY_IN_MIN_SEC + SURVEY_IN_EXTRA_WAIT_SEC
            print(f"[WAITING] Waiting {SURVEY_IN_MIN_SEC} seconds for Survey-in.")
            print("[WARNING] Absolutely do not move the base antenna during this time.")
            print("[INFO] RTCM transfer will automatically start when waiting is over.")
            time.sleep(wait_sec)
            print("[OK] Survey-in waiting time completed. RTCM transfer starting.")
    else:
        print("[INFO] Base GPS configuration skipped.")

    stop_event = threading.Event()

    t1 = threading.Thread(
        target=gps_to_radio_rtcm,
        args=(gps_ser, radio_ser, stop_event),
        daemon=True
    )

    t2 = threading.Thread(
        target=radio_to_console,
        args=(radio_ser, stop_event),
        daemon=True
    )

    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[EXIT] Program stopping...")

    finally:
        stop_event.set()
        time.sleep(0.3)
        gps_ser.close()
        radio_ser.close()
        print("[EXIT] Serial ports closed.")


if __name__ == "__main__":
    main()