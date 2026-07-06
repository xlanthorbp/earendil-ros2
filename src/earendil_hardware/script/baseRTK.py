#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BASE TARAFI

LC29HEA Base GPS -> 3DR SiK Telemetry Radio -> Rover

Görev:
1. Base LC29HEA'yı base moduna alır.
2. Survey-in veya fixed ECEF base konumu ayarlar.
3. GPS'ten gelen RTCM3 paketlerini ayıklar.
4. Sadece geçerli RTCM3 frame'lerini 3DR radyoya gönderir.
5. Rover'dan gelen kısa durum mesajlarını ekrana basar.

Not:
- 3DR SiK radyo ayarlarında MAVLINK=0 önerilir.
- İki radyoda NETID, baud ve air speed aynı olmalıdır.
"""

import serial
import threading
import time
from collections import Counter


# =======================================================
# 1) PORT VE BAUD AYARLARI
# =======================================================

BASE_GPS_PORT = "/dev/ttyUSB0"       # Windows örnek: COM8
BASE_RADIO_PORT = "/dev/ttyUSB1"    # Windows örnek: COM12

# Ubuntu örnek:
# BASE_GPS_PORT = "/dev/ttyUSB0"
# BASE_RADIO_PORT = "/dev/ttyUSB1"

GPS_BAUD = 460800
RADIO_BAUD = 57600

SERIAL_TIMEOUT = 0.05


# =======================================================
# 2) BASE KONFİGÜRASYONU
# =======================================================

CONFIGURE_BASE_ON_START = True

# "survey_in" veya "fixed_ecef"
# Base anteni her kurulumda farklı noktaya konuyorsa survey_in kullanılmalı.
BASE_POSITION_MODE = "survey_in"

# Survey-in ayarı
# Bu sürümde sistem açılışta 5 dakika base konumunu ortalar,
# ardından RTCM aktarım thread'lerini başlatır.
SURVEY_IN_MIN_SEC = 400   # 5 dakika
SURVEY_IN_ACC_M = 0.5        # hedef survey-in doğruluğu: 0.5 m
WAIT_SURVEY_IN_BEFORE_RTCM = True
SURVEY_IN_EXTRA_WAIT_SEC = 10

# Fixed ECEF kullanacaksan bunları doldur.
# Değerler metre cinsinden WGS84 ECEF X/Y/Z olmalı.
FIXED_ECEF_X = 0.0
FIXED_ECEF_Y = 0.0
FIXED_ECEF_Z = 0.0

# RTCM mesajlarını açma
ENABLE_RTCM_1005 = True
ENABLE_RTCM_MSM = True

# Eğer RF bandwidth yetmezse MSM7 yerine alıcının default MSM4 ayarı daha stabil olabilir.
# Şimdilik senin eski kodundaki PAIR432 ve PAIR434 mantığını koruyoruz.


# =======================================================
# 3) NMEA / QUECTEL KOMUT CHECKSUM
# =======================================================

def checksum(nmea_body: str) -> str:
    cs = 0
    for ch in nmea_body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def send_cmd(ser: serial.Serial, raw_cmd: str, wait: float = 0.25) -> None:
    """
    Quectel/LC29H komutunu checksum'lı gönderir.
    Örnek input:
        PQTMCFGRCVRMODE,W,2
    Gönderilen:
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
            print(f"[GPS CEVAP] {resp}")
        else:
            print(f"[GPS CEVAP YOK] {body}")
    except Exception as e:
        print(f"[GPS CEVAP OKUMA HATASI] {e}")


def configure_base_gps(gps_ser: serial.Serial) -> None:
    print("\n=== Base GPS konfigürasyonu başlıyor ===")

    # 1) Base station mode
    send_cmd(gps_ser, "PQTMCFGRCVRMODE,W,2")

    # 2) Base position
    if BASE_POSITION_MODE == "survey_in":
        cmd = f"PQTMCFGSVIN,W,1,{SURVEY_IN_MIN_SEC},{SURVEY_IN_ACC_M},0,0,0"
        send_cmd(gps_ser, cmd)
        print(f"[BASE MODE] Survey-in aktif: {SURVEY_IN_MIN_SEC}s, hedef doğruluk {SURVEY_IN_ACC_M}m")

    elif BASE_POSITION_MODE == "fixed_ecef":
        if FIXED_ECEF_X == 0.0 and FIXED_ECEF_Y == 0.0 and FIXED_ECEF_Z == 0.0:
            print("[UYARI] fixed_ecef seçili ama ECEF değerleri 0.0. Bu doğru değildir.")
        cmd = f"PQTMCFGSVIN,W,2,0,0,{FIXED_ECEF_X},{FIXED_ECEF_Y},{FIXED_ECEF_Z}"
        send_cmd(gps_ser, cmd)
        print("[BASE MODE] Fixed ECEF aktif.")

    else:
        print("[UYARI] BASE_POSITION_MODE hatalı. Survey-in/fixed_ecef dışında değer girilmiş.")

    # 3) RTCM 1005 aç
    if ENABLE_RTCM_1005:
        send_cmd(gps_ser, "PAIR434,1")

    # 4) RTCM MSM aç
    if ENABLE_RTCM_MSM:
        send_cmd(gps_ser, "PAIR432,1")

    print("=== Base GPS konfigürasyonu bitti ===\n")


# =======================================================
# 4) RTCM3 PARSER
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
# 5) BASE GPS -> RADIO RTCM AKTARIMI
# =======================================================

def gps_to_radio_rtcm(gps_ser: serial.Serial, radio_ser: serial.Serial, stop_event: threading.Event):
    print("[AKTARIM] Base GPS -> Radio RTCM başladı.")

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
                    print("[BASE TX] RTCM yok. Base GPS RTCM üretiyor mu kontrol et.")

                byte_count = 0
                frame_count = 0
                msg_counter.clear()
                last_report = now

        except Exception as e:
            print(f"[HATA] GPS -> Radio RTCM aktarım hatası: {e}")
            time.sleep(0.2)


# =======================================================
# 6) ROVER -> BASE DURUM MESAJI OKUMA
# =======================================================

def radio_to_console(radio_ser: serial.Serial, stop_event: threading.Event):
    print("[DINLEME] Rover durum mesajı dinleme başladı.")

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

            # Buffer şişmesin.
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
            print(f"[HATA] Radio console okuma hatası: {e}")
            time.sleep(0.2)


# =======================================================
# 7) MAIN
# =======================================================

def main():
    print("=== LC29HEA BASE -> 3DR SiK RTCM AKTARIM ===")

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

        print(f"[OK] Base GPS bağlandı: {BASE_GPS_PORT} @ {GPS_BAUD}")
        print(f"[OK] Base Radio bağlandı: {BASE_RADIO_PORT} @ {RADIO_BAUD}")

    except Exception as e:
        print(f"[BAGLANTI HATASI] {e}")
        return

    if CONFIGURE_BASE_ON_START:
        configure_base_gps(gps_ser)

        if BASE_POSITION_MODE == "survey_in" and WAIT_SURVEY_IN_BEFORE_RTCM:
            wait_sec = SURVEY_IN_MIN_SEC + SURVEY_IN_EXTRA_WAIT_SEC
            print(f"[BEKLEME] Survey-in için {SURVEY_IN_MIN_SEC} saniye bekleniyor.")
            print("[UYARI] Bu sürede base antenini kesinlikle oynatma.")
            print("[INFO] Bekleme bitince RTCM aktarımı otomatik başlayacak.")
            time.sleep(wait_sec)
            print("[OK] Survey-in bekleme süresi tamamlandı. RTCM aktarımı başlatılıyor.")
    else:
        print("[INFO] Base GPS konfigürasyonu atlandı.")

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
        print("\n[CIKIS] Program durduruluyor...")

    finally:
        stop_event.set()
        time.sleep(0.3)
        gps_ser.close()
        radio_ser.close()
        print("[CIKIS] Seri portlar kapatıldı.")


if __name__ == "__main__":
    main()