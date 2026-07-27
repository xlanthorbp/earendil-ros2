#!/usr/bin/env python3
"""
=============================================================================
BASE RTK GPS - TEK SEFERLİK KURULUM & İZLEME SCRIPTİ (Quectel LC29H)
=============================================================================
Bu script Quectel LC29HEA GPS modülünü "Base (Sabit İstasyon)" moduna almak,
Survey-In (otomatik konum ortalaması) sürecini başlatmak ve yapılan ayarları
modülün dâhili kalıcı Flash hafızasına ($PQTMSAVEPAR) yazmak için kullanılır.

Donanım Yapısı:
- Base GPS USB kablosu ile bilgisayara takılır (komut ve izleme için).
- Base GPS'in UART TX pini, doğrudan Base RF modülünün RX pinine bağlıdır.
- Arduino 5V ve GND pinlerinden breadboard üzerinden RF ve GPS'e güç verilir.
- Flash'a kaydetme tamamlandıktan sonra bilgisayar kablosu SÖKÜLEBİLİR.
  Base sistemi Arduino 5V/GND gücüyle bilgisayarsız (standalone) çalışır.
=============================================================================
"""

import time
import serial
from typing import Optional


# ============================================================
# KURULUM AYARLARI
# ============================================================

GPS_PORT = "/dev/ttyUSB0"   # Base LC29H GPS portu (USB)
GPS_BAUD = 115200          # GPS USB iletişim baud hızı

# Survey-In Ayarları
SURVEY_SECONDS = 600       # Hedef Survey-In süresi (saniye - 10 dakika = 600sn)
SURVEY_ACC_M = 1.0         # Hedef Survey-In 3D konum hassasiyeti (metre)

# RF Modülünüzün UART Baud Hızı (LC29H UART TX pini bu hızda yayın yapacak)
RF_UART_BAUD = 115200

# Kesin ECEF koordinatları biliniyorsa yazılır. Bilinmiyorsa None bırakılır (Survey-In yapılır).
ECEF_X = None
ECEF_Y = None
ECEF_Z = None


# ============================================================
# YARDIMCI FONKSİYONLAR
# ============================================================

def nmea_checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def nmea_latlon_to_decimal(value: str, hemi: str):
    if not value or not hemi:
        return None
    try:
        raw = float(value)
        degrees = int(raw // 100)
        minutes = raw - degrees * 100
        dec = degrees + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None


def parse_gga_latlon(line: str):
    try:
        body = line[1:].split("*")[0]
        parts = body.split(",")
        if not parts[0].endswith("GGA"):
            return None, None
        lat = nmea_latlon_to_decimal(parts[2], parts[3])
        lon = nmea_latlon_to_decimal(parts[4], parts[5])
        return lat, lon
    except Exception:
        return None, None


def make_map_link(lat: float, lon: float) -> str:
    return f"https://www.google.com/maps?q={lat:.8f},{lon:.8f}"


def make_cmd(body: str) -> bytes:
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def send_cmd(ser: serial.Serial, body: str, wait: float = 0.3) -> None:
    cmd = make_cmd(body)
    print(f"[GPS KOMUT] Send -> {cmd.decode().strip()}")
    ser.write(cmd)
    ser.flush()
    time.sleep(wait)


class NMEALineParser:
    """GPS seri akışı içinden NMEA satırlarını ayıklar."""
    def __init__(self):
        self.buf = bytearray()
        self.collecting = False

    def feed(self, data: bytes):
        lines = []
        for b in data:
            if b == ord("$"):
                self.buf.clear()
                self.buf.append(b)
                self.collecting = True
            elif self.collecting:
                if b == 10:  # LF
                    line = self.buf.decode("ascii", errors="ignore").strip()
                    self.buf.clear()
                    self.collecting = False
                    if line:
                        lines.append(line)
                elif b != 13:  # CR hariç ekle
                    if len(self.buf) < 300:
                        self.buf.append(b)
                    else:
                        self.buf.clear()
                        self.collecting = False
        return lines


# ============================================================
# LC29H BASE CONFIGURATION & FLASH SAVE
# ============================================================

def configure_base(
    gps: serial.Serial,
    survey_seconds: int,
    survey_acc_m: float,
    ecef_x: Optional[str] = None,
    ecef_y: Optional[str] = None,
    ecef_z: Optional[str] = None,
) -> None:
    print("\n==================================================")
    print("[BASE SETUP] LC29H Base mod konfigürasyonu gönderiliyor...")
    print("==================================================")

    # 1. Modülü BASE alıcı moduna geçir
    send_cmd(gps, "PQTMCFGRCVRMODE,W,2")

    # 2. Donanımsal UART1 çıkış hızını RF modülü ile eşitle (115200)
    send_cmd(gps, f"PAIR864,0,0,{RF_UART_BAUD}")

    # 3. Survey-In durum raporlama mesajını ($PQTMSVINSTATUS) aç
    send_cmd(gps, "PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1")

    # 4. Base Mod / Survey-In Tipi
    if ecef_x is not None and ecef_y is not None and ecef_z is not None:
        send_cmd(gps, f"PQTMCFGSVIN,W,2,0,0,{ecef_x},{ecef_y},{ecef_z}")
        print("[BASE SETUP] Sabit ECEF koordinatı ayarlandı.")
    else:
        send_cmd(gps, f"PQTMCFGSVIN,W,1,{survey_seconds},{survey_acc_m},0,0,0")
        print(f"[BASE SETUP] Survey-In başlatıldı -> Süre: {survey_seconds} sn | Hedef Acc: {survey_acc_m} m")

    # 5. RTCM3 MSM gözlem mesajlarını aç (PAIR432,1)
    send_cmd(gps, "PAIR432,1")

    # 6. RTCM3 1005 Base İstasyon Anten Konum mesajını aç (PAIR434,1)
    send_cmd(gps, "PAIR434,1")

    # 7. GGA mesajını açık tut (Durum izleme için)
    send_cmd(gps, "PAIR062,0,01")

    # 8. KRİTİK ADIM: Tüm parametreleri kalıcı Flash hafızaya kaydet
    send_cmd(gps, "PQTMSAVEPAR", 1.0)

    print("\n[BASE SETUP] Konfigürasyon komutları başarıyla gönderildi ve FLASH hafızaya kaydedildi ($PQTMSAVEPAR)!")
    print("[BASE SETUP] Şimdi Survey-In takibi başlatılıyor...\n")


# ============================================================
# SURVEY-IN İZLEME VE DURUM EKRANI
# ============================================================

def monitor_survey(gps: serial.Serial, target_seconds: int) -> None:
    parser = NMEALineParser()
    start_time = time.time()
    last_print = 0

    print("==================================================")
    print(f"[SURVEY-IN İZLEME] Süreç başlatıldı.")
    print("[SURVEY-IN İZLEME] GPS dahili ortalamayı hesaplıyor.")
    print("==================================================")

    last_lat, last_lon = None, None

    try:
        while True:
            now = time.time()
            elapsed = int(now - start_time)

            data = gps.read(4096)
            if data:
                for line in parser.feed(data):
                    if line.startswith("$PQTMSVINSTATUS"):
                        print(f"[SURVEY-IN BİLGİ] {line}")
                        parts = line.split(",")
                        if len(parts) >= 3 and parts[2] == "1":
                            print("\n==================================================")
                            print("🎉 SURVEY-IN BAŞARIYLA TAMAMLANDI!")
                            print("Base istasyonu hassas konumunu sabitledi ve RTCM3 yayınlıyor.")
                            if last_lat is not None and last_lon is not None:
                                print(f"📍 Base Sabit Konum Google Harita: {make_map_link(last_lat, last_lon)}")
                            print("==================================================")
                            return

                    elif "GGA" in line:
                        lat, lon = parse_gga_latlon(line)
                        if lat is not None and lon is not None:
                            last_lat, last_lon = lat, lon
                        if now - last_print >= 5.0:
                            if lat is not None and lon is not None:
                                map_url = make_map_link(lat, lon)
                                print(f"[BASE KONUM] Enlem: {lat:.8f}, Boylam: {lon:.8f} | Google Harita: {map_url}")
                            else:
                                print(f"[BASE GGA FIX] {line}")

            if now - last_print >= 10.0:
                last_print = now
                print(f"[SURVEY-IN] Geçen Süre: {elapsed} sn / Hedef: {target_seconds} sn")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[İPTAL] Kullanıcı tarafından durduruldu.")

    print("\n==================================================")
    print("ℹ️ NOT: Ayarlar Flash hafızaya kaydedildiği için ($PQTMSAVEPAR)")
    print("artık bilgisayar kablosunu sökebilirsiniz!")
    print("Base modülü Arduino 5V/GND gücü ile doğrudan çalışmaya devam edecektir.")
    print("==================================================\n")


# ============================================================
# MAIN
# ============================================================

def main() -> None:
    print("==================================================")
    print(" Quectel LC29H Base RTK GPS Tek Seferlik Setup")
    print("==================================================")
    print(f"[PORT] GPS USB Port: {GPS_PORT} @ {GPS_BAUD} baud")

    try:
        gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"❌ Port açılamadı: {e}")
        print("Lütfen USB kablosunun takılı ve port isminin doğru olduğunu kontrol edin.")
        return

    time.sleep(1.0)

    # Base konfigürasyonunu yükle ve Flash'a kaydet
    configure_base(
        gps=gps,
        survey_seconds=SURVEY_SECONDS,
        survey_acc_m=SURVEY_ACC_M,
        ecef_x=ECEF_X,
        ecef_y=ECEF_Y,
        ecef_z=ECEF_Z,
    )

    # Survey-In sürecini ekranda izle
    monitor_survey(gps, SURVEY_SECONDS)

    gps.close()


if __name__ == "__main__":
    main()
