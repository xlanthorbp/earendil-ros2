#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROVER TARAFI

Base -> 3DR SiK Radio -> Rover LC29HEA

Görev:
1. 3DR radyodan gelen RTCM3 paketlerini okur.
2. Sadece geçerli RTCM3 frame'lerini Rover LC29HEA GPS'e yazar.
3. Rover GPS'ten GGA NMEA satırlarını okur.
4. RTK durumunu, konumu, uydu sayısını, HDOP ve correction age bilgisini ekrana basar.
5. İsterse Base'e kısa ASCII durum mesajı yollar.

Not:
- RF hattını boğmamak için Base'e giden durum mesajı 1 Hz ve kısa tutulmuştur.
- Emoji, harita linki, uzun text RF hattına gönderilmez.
"""

import serial
import threading
import time
from collections import Counter


# =======================================================
# 1) PORT VE BAUD AYARLARI
# =======================================================

ROVER_GPS_PORT = "/dev/ttyUSB1"       # Windows örnek: COM9
ROVER_RADIO_PORT = "/dev/ttyUSB2"    # Windows örnek: COM11

# Ubuntu örnek:
# ROVER_GPS_PORT = "/dev/ttyUSB0"
# ROVER_RADIO_PORT = "/dev/ttyUSB1"

GPS_BAUD = 460800
RADIO_BAUD = 57600

SERIAL_TIMEOUT = 0.05


# =======================================================
# 2) ROVER STATUS AYARI
# =======================================================

SEND_STATUS_TO_BASE = False
STATUS_SEND_INTERVAL = 1.0    # saniye

PRINT_INTERVAL = 1.0          # saniye


# =======================================================
# 3) RTCM3 PARSER
# =======================================================

def crc24q(data: bytes) -> int:
    """
    RTCM3 CRC24Q hesabı.
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
    RTCM message type ilk 12 bittir.
    """
    payload = frame[3:-3]
    if len(payload) < 2:
        return -1
    return (payload[0] << 4) | (payload[1] >> 4)


def extract_rtcm3_frames(buffer: bytearray):
    """
    Buffer içinden geçerli RTCM3 frame'lerini çıkarır.
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

        # RTCM header'da 2. byte'ın üst 6 biti reserved = 0 olmalı.
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
            # Yanlış D3 yakalandıysa bir byte ilerle.
            buffer = buffer[start + 1:]


# =======================================================
# 4) NMEA GGA PARSER
# =======================================================

def nmea2dec(coord: str, direction: str) -> float:
    """
    NMEA DDMM.MMMMM / DDDMM.MMMMM formatını decimal dereceye çevirir.
    """
    if not coord:
        raise ValueError("Boş koordinat")

    dot = coord.index(".")
    deg = float(coord[:dot - 2])
    mins = float(coord[dot - 2:])
    dec = deg + mins / 60.0

    if direction in ("S", "W"):
        dec = -dec

    return dec


def parse_gga(line: str):
    """
    GGA satırını parse eder.

    Örnek:
    $GNGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,diff_age,diff_station*CS
    """
    if "GGA," not in line:
        return None

    parts = line.split(",")

    if len(parts) < 10:
        return None

    try:
        utc_time = parts[1]
        lat_raw = parts[2]
        lat_dir = parts[3]
        lon_raw = parts[4]
        lon_dir = parts[5]
        quality = parts[6]
        satellites = parts[7]
        hdop = parts[8]
        altitude = parts[9]
        diff_age = parts[13] if len(parts) > 13 else ""

        lat = None
        lon = None

        if quality != "0" and lat_raw and lon_raw:
            lat = nmea2dec(lat_raw, lat_dir)
            lon = nmea2dec(lon_raw, lon_dir)

        return {
            "utc_time": utc_time,
            "lat": lat,
            "lon": lon,
            "quality": quality,
            "satellites": satellites,
            "hdop": hdop,
            "altitude": altitude,
            "diff_age": diff_age,
            "raw": line.strip()
        }

    except Exception:
        return None


def fix_name(quality: str) -> str:
    names = {
        "0": "INVALID",
        "1": "SPS",
        "2": "DGPS",
        "4": "RTK_FIXED",
        "5": "RTK_FLOAT",
        "6": "DEAD_RECKONING"
    }
    return names.get(quality, "UNKNOWN")


# =======================================================
# 5) RADIO -> ROVER GPS RTCM AKTARIMI
# =======================================================

def radio_to_gps_rtcm(radio_ser: serial.Serial, gps_ser: serial.Serial, stop_event: threading.Event):
    print("[AKTARIM] Radio -> Rover GPS RTCM başladı.")

    buffer = bytearray()
    last_report = time.time()
    byte_count = 0
    frame_count = 0
    msg_counter = Counter()

    while not stop_event.is_set():
        try:
            data = radio_ser.read(radio_ser.in_waiting or 1)

            if data:
                buffer.extend(data)
                buffer, frames = extract_rtcm3_frames(buffer)

                for frame in frames:
                    gps_ser.write(frame)
                    gps_ser.flush()

                    byte_count += len(frame)
                    frame_count += 1
                    msg_counter[rtcm_message_type(frame)] += 1

            now = time.time()
            if now - last_report >= 1.0:
                if frame_count > 0:
                    top_msgs = ", ".join(
                        f"{msg}:{cnt}" for msg, cnt in msg_counter.most_common(6)
                    )
                    print(f"[ROVER RTCM RX] {byte_count} byte/s | {frame_count} frame/s | msg: {top_msgs}")
                else:
                    print("[ROVER RTCM RX] RTCM yok. RF link veya Base çıkışı kontrol et.")

                byte_count = 0
                frame_count = 0
                msg_counter.clear()
                last_report = now

        except Exception as e:
            print(f"[HATA] Radio -> GPS RTCM aktarım hatası: {e}")
            time.sleep(0.2)


# =======================================================
# 6) ROVER GPS GGA MONITOR
# =======================================================

def monitor_rover_gps(gps_ser: serial.Serial, radio_ser: serial.Serial, stop_event: threading.Event):
    print("[DINLEME] Rover GPS GGA dinleme başladı.")

    last_print = 0.0
    last_status_send = 0.0

    while not stop_event.is_set():
        try:
            raw_line = gps_ser.readline()

            if not raw_line:
                continue

            line = raw_line.decode("ascii", errors="ignore").strip()

            if not line:
                continue

            gga = parse_gga(line)

            if gga is None:
                continue

            quality = gga["quality"]
            fix = fix_name(quality)
            sats = gga["satellites"] or "0"
            hdop = gga["hdop"] or ""
            altitude = gga["altitude"] or ""
            diff_age = gga["diff_age"] or ""

            lat = gga["lat"]
            lon = gga["lon"]

            now = time.time()

            # -------------------------------
            # 1) Ekrana basma
            # -------------------------------
            if now - last_print >= PRINT_INTERVAL:
                if lat is not None and lon is not None:
                    osm = (
                        f"https://www.openstreetmap.org/"
                        f"?mlat={lat:.8f}&mlon={lon:.8f}"
                        f"#map=19/{lat:.8f}/{lon:.8f}"
                    )

                    print(
                        "\n"
                        f"[GGA] fix={fix} | q={quality} | "
                        f"lat={lat:.8f} | lon={lon:.8f} | "
                        f"alt={altitude}m | sats={sats} | hdop={hdop} | diff_age={diff_age}s\n"
                        f"[MAP] {osm}"
                    )
                else:
                    print(
                        "\n"
                        f"[GGA] fix={fix} | q={quality} | "
                        f"sats={sats} | hdop={hdop} | GPS fix bekleniyor..."
                    )

                last_print = now

            # -------------------------------
            # 2) Base'e kısa status gönderme
            # -------------------------------
            if SEND_STATUS_TO_BASE and now - last_status_send >= STATUS_SEND_INTERVAL:
                try:
                    if lat is not None and lon is not None:
                        status = (
                            f"ROVER,"
                            f"q={quality},"
                            f"fix={fix},"
                            f"lat={lat:.8f},"
                            f"lon={lon:.8f},"
                            f"alt={altitude},"
                            f"sats={sats},"
                            f"hdop={hdop},"
                            f"age={diff_age}\r\n"
                        )
                    else:
                        status = (
                            f"ROVER,"
                            f"q={quality},"
                            f"fix={fix},"
                            f"lat=,"
                            f"lon=,"
                            f"alt=,"
                            f"sats={sats},"
                            f"hdop={hdop},"
                            f"age={diff_age}\r\n"
                        )

                    radio_ser.write(status.encode("ascii", errors="ignore"))
                    radio_ser.flush()
                    last_status_send = now

                except Exception as e:
                    print(f"[HATA] Rover status gönderilemedi: {e}")

        except Exception as e:
            print(f"[HATA] Rover GPS monitor hatası: {e}")
            time.sleep(0.2)


# =======================================================
# 7) MAIN
# =======================================================

def main():
    print("=== 3DR SiK -> LC29HEA ROVER RTCM AKTARIM ===")

    try:
        gps_ser = serial.Serial(
            ROVER_GPS_PORT,
            GPS_BAUD,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1
        )

        radio_ser = serial.Serial(
            ROVER_RADIO_PORT,
            RADIO_BAUD,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1
        )

        gps_ser.reset_input_buffer()
        gps_ser.reset_output_buffer()
        radio_ser.reset_input_buffer()
        radio_ser.reset_output_buffer()

        print(f"[OK] Rover GPS bağlandı: {ROVER_GPS_PORT} @ {GPS_BAUD}")
        print(f"[OK] Rover Radio bağlandı: {ROVER_RADIO_PORT} @ {RADIO_BAUD}")

    except Exception as e:
        print(f"[BAGLANTI HATASI] {e}")
        return

    stop_event = threading.Event()

    t1 = threading.Thread(
        target=radio_to_gps_rtcm,
        args=(radio_ser, gps_ser, stop_event),
        daemon=True
    )

    t2 = threading.Thread(
        target=monitor_rover_gps,
        args=(gps_ser, radio_ser, stop_event),
        daemon=True
    )

    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[CIKIS] Program durduruluyor...")

    finally:
        stop_event.set()
        time.sleep(0.3)
        gps_ser.close()
        radio_ser.close()
        print("[CIKIS] Seri portlar kapatıldı.")


if __name__ == "__main__":
    main()