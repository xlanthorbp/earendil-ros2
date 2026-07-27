#!/usr/bin/env python3
"""
=============================================================================
BASE RTK GPS - HIZLI TEST SCRIPTİ (SURVEY-IN BEKLEMEDEN VERİ AKIŞ TESTİ)
=============================================================================
Bu script Quectel LC29H modülünü Base moduna alır ve Survey-In süresini 
beklemeden (atlayarak) HIZLICA RTCM3 verisi üretip RF üzerinden yaymaya
başlamasını sağlar.

Amaç:
- RF vericisinin yayın yapıp yapmadığını test etmek,
- Rover RF alıcısının ve ROS 2 düğümünün RTCM verisini alıp almadığını
  anında (5-10 saniye içinde) doğrulamak.
=============================================================================
"""

import time
import serial

# ============================================================
# TEST AYARLARI
# ============================================================

GPS_PORT = "/dev/ttyUSB0"   # Base LC29H GPS portu (USB)
GPS_BAUD = 115200          # GPS USB iletişim baud hızı
RF_UART_BAUD = 115200      # RF modülü baud hızı


def nmea_checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def make_cmd(body: str) -> bytes:
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def send_cmd(ser: serial.Serial, body: str, wait: float = 0.4) -> None:
    cmd = make_cmd(body)
    print(f"[TEST KOMUT] Send -> {cmd.decode().strip()}")
    ser.write(cmd)
    ser.flush()
    time.sleep(wait)
    # Gelen yanıtı (OK / ERROR) oku ve ekrana bas
    while ser.in_waiting:
        resp = ser.readline().decode("ascii", errors="ignore").strip()
        if resp:
            print(f"   [GPS YANIT] <- {resp}")


def main() -> None:
    print("==================================================")
    print(" LC29H Base RTK Hızlı Test Scripti (Survey-In Yok)")
    print("==================================================")
    print(f"[PORT] GPS USB Port: {GPS_PORT} @ {GPS_BAUD} baud")

    try:
        gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"❌ Port açılamadı: {e}")
        return

    time.sleep(1.0)

    print("\n[HIZLI TEST] LC29H anında RTCM3 yayın moduna alınıyor...")

    # 1. Alıcı modunu sıfırla (Standby/Normal moda alıp ardından Base moduna geçirerek Survey-In'i resetle)
    send_cmd(gps, "PQTMCFGRCVRMODE,W,0", wait=0.5)
    send_cmd(gps, "PQTMCFGRCVRMODE,W,2", wait=0.5)

    # 2. DONANIM_UART1 (RF Çıkışı) hızını 115200 yap
    send_cmd(gps, f"PAIR864,0,0,{RF_UART_BAUD}", wait=0.3)

    # 3. LC29H izin verilen minimum süreyi (120 sn) ve maksimum toleransı (10.0m) ayarla
    send_cmd(gps, "PQTMCFGSVIN,W,1,120,10.0,0,0,0", wait=0.5)

    # 4. RTCM3 MSM gözlem ve 1005 mesajlarını anında aç
    send_cmd(gps, "PAIR432,1")
    send_cmd(gps, "PAIR434,1")

    # 5. GGA ve PQTM mesajlarını aç
    send_cmd(gps, "PAIR062,0,01")

    # 6. Ayarları kaydet
    send_cmd(gps, "PQTMSAVEPAR", 1.0)

    print("\n==================================================")
    print("⚡ HIZLI TEST MODU AKTİF!")
    print("LC29H uyduları görür görmez RTCM3 verisini RF'e basacaktır.")
    print("Şimdi Rover tarafındaki ROS 2 düğümünü çalıştırıp test edebilirsiniz.")
    print("Durdurmak için Ctrl+C basın.")
    print("==================================================\n")

    try:
        while True:
            line = gps.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("$G") or line.startswith("$PQTM"):
                print(f"[BASE CANLI] {line}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[TEST BİTTİ] Test scripti kapatıldı.")
        gps.close()


if __name__ == "__main__":
    main()
