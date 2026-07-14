import serial
import time
from collections import Counter
from typing import Optional


# ============================================================
# BASE AYARLARI
# ============================================================

GPS_PORT = "/dev/ttyUSB0"   # Base LC29HEA GPS portu
RF_PORT = "/dev/ttyUSB1"    # Base 3DR RF portu

GPS_BAUD = 460800
RF_BAUD = 57600

CONFIGURE_BASE = True

# 8 dakika = 480 saniye
WAIT_BEFORE_BRIDGE_SECONDS = 480

# Survey-in ayarı
# 8 dakika bekleyeceğimiz için survey süresini de 480 yaptım.
SURVEY_SECONDS = 480

# 5 cm hedef için 15 m çok gevşekti.
# İlk düzgün test için 1.0 m daha mantıklı.
# Daha sıkı istersen 0.5 yapabilirsin ama survey-in tamamlanması zorlaşabilir.
SURVEY_ACC_M = 1.0

# Eğer base'in kesin ECEF koordinatını biliyorsan buraya yaz.
# Bilmiyorsan None kalsın, survey-in kullanılır.
ECEF_X = None
ECEF_Y = None
ECEF_Z = None


# ============================================================
# YARDIMCI FONKSIYONLAR
# ============================================================

def nmea_checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def make_cmd(body: str) -> bytes:
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def send_cmd(ser: serial.Serial, body: str, wait: float = 0.25) -> None:
    cmd = make_cmd(body)
    print(f"[GPS CMD] {cmd.decode().strip()}")
    ser.write(cmd)
    ser.flush()
    time.sleep(wait)


class NMEALineParser:
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

                elif b == 13:  # CR
                    pass

                else:
                    if len(self.buf) < 300:
                        self.buf.append(b)
                    else:
                        self.buf.clear()
                        self.collecting = False

        return lines


class RTCMExtractor:
    """
    RTCM3 frame yapısı:
    0xD3 + 2 byte length + payload + 3 byte CRC

    Bu sınıf stream içinden RTCM frame'lerini ayıklar.
    """

    def __init__(self):
        self.buf = bytearray()
        self.counts = Counter()

    def feed(self, data: bytes):
        frames = []
        self.buf.extend(data)

        while True:
            idx = self.buf.find(b"\xD3")

            if idx < 0:
                if len(self.buf) > 4096:
                    self.buf.clear()
                return frames

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 3:
                return frames

            length = ((self.buf[1] & 0x03) << 8) | self.buf[2]

            if length > 1023:
                del self.buf[0]
                continue

            total_len = 3 + length + 3

            if len(self.buf) < total_len:
                return frames

            frame = bytes(self.buf[:total_len])
            del self.buf[:total_len]

            if length >= 2:
                payload = frame[3:3 + length]
                msg_id = (payload[0] << 4) | (payload[1] >> 4)
                self.counts[msg_id] += 1

            frames.append(frame)

        return frames


# ============================================================
# LC29HEA BASE CONFIG
# ============================================================

def configure_base(
    gps: serial.Serial,
    survey_seconds: int,
    survey_acc_m: float,
    ecef_x: Optional[str],
    ecef_y: Optional[str],
    ecef_z: Optional[str],
) -> None:
    print("\n[BASE CONFIG] LC29HEA base ayarlari gonderiliyor...")

    # Receiver mode: BASE
    send_cmd(gps, "PQTMCFGRCVRMODE,W,2")
    send_cmd(gps, "PQTMSAVEPAR", 0.5)

    # Survey-in status mesajını aç
    send_cmd(gps, "PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1")

    if ecef_x is not None and ecef_y is not None and ecef_z is not None:
        # Kesin base koordinatı girilecekse
        send_cmd(gps, f"PQTMCFGSVIN,W,2,0,0,{ecef_x},{ecef_y},{ecef_z}")
        print("[BASE CONFIG] Sabit ECEF base koordinati girildi.")
    else:
        # Survey-in başlat
        send_cmd(gps, f"PQTMCFGSVIN,W,1,{survey_seconds},{survey_acc_m},0,0,0")
        print(f"[BASE CONFIG] Survey-in baslatildi: {survey_seconds} sn, hedef acc {survey_acc_m} m")

    # RTCM3 MSM observation mesajlarını aç
    send_cmd(gps, "PAIR432,1")

    # RTCM3 1005 base antenna position mesajını aç
    send_cmd(gps, "PAIR434,1")

    # GGA açık kalsın; base durumunu görmek için
    send_cmd(gps, "PAIR062,0,01")

    send_cmd(gps, "PQTMSAVEPAR", 0.5)

    print("[BASE CONFIG] Komutlar gonderildi.")
    print("[BASE CONFIG] Simdi 8 dakika survey-in / stabilizasyon beklemesi yapilacak.")
    print("[BASE CONFIG] Bu surede RF'e RTCM gonderilmeyecek.\n")


# ============================================================
# 8 DAKIKA BEKLEME
# ============================================================

def wait_before_bridge(gps: serial.Serial, wait_seconds: int) -> None:
    nmea = NMEALineParser()
    rtcm = RTCMExtractor()

    start = time.time()
    last_status_print = 0
    total_gps_bytes = 0

    print("==================================================")
    print(f"[WAIT] {wait_seconds} saniye bekleme basladi.")
    print("[WAIT] GPS okunuyor ama RF'e veri gonderilmiyor.")
    print("==================================================")

    while True:
        now = time.time()
        elapsed = int(now - start)
        remaining = wait_seconds - elapsed

        if remaining <= 0:
            print("\n[WAIT] Bekleme tamamlandi. RTCM -> RF iletisim baslatiliyor.\n")
            break

        data = gps.read(4096)

        if data:
            total_gps_bytes += len(data)

            # Bu sırada RTCM'i sadece sayıyoruz, RF'e göndermiyoruz.
            rtcm.feed(data)

            for line in nmea.feed(data):
                if line.startswith("$PQTMSVINSTATUS"):
                    print(f"[SURVEY] {line}")
                elif line.startswith("$G") and "GGA" in line:
                    print(f"[BASE GGA] {line}")
                elif line.startswith("$PQTM"):
                    print(f"[BASE PQTM] {line}")

        # Her 10 saniyede bir kısa durum bas
        if now - last_status_print >= 10.0:
            last_status_print = now

            if rtcm.counts:
                counts_text = ", ".join(
                    f"{msg}:{cnt}" for msg, cnt in sorted(rtcm.counts.items())
                )
            else:
                counts_text = "henuz RTCM yok"

            print(
                f"[WAIT] Kalan: {remaining} sn | "
                f"gps_bytes={total_gps_bytes} | "
                f"RTCM={{ {counts_text} }}"
            )


# ============================================================
# BASE BRIDGE
# ============================================================

def bridge_base(gps: serial.Serial, rf: serial.Serial) -> None:
    rtcm = RTCMExtractor()
    nmea = NMEALineParser()

    total_gps_bytes = 0
    total_rf_bytes = 0
    last_print = time.time()

    print("[BASE] GPS -> RF RTCM koprusu basladi.")
    print("[BASE] GPS'ten sadece RTCM 0xD3 frame'leri RF'e gonderilecek.")
    print("[BASE] Ctrl+C ile cik.\n")

    while True:
        data = gps.read(4096)

        if data:
            total_gps_bytes += len(data)

            # NMEA/PQTM durum satırlarını terminalde göster
            for line in nmea.feed(data):
                if line.startswith("$PQTMSVINSTATUS"):
                    print(f"[SURVEY] {line}")
                elif line.startswith("$G") or line.startswith("$PQTM"):
                    print(f"[BASE GPS] {line}")

            # RTCM frame'lerini ayıkla ve RF'e gönder
            frames = rtcm.feed(data)

            for frame in frames:
                rf.write(frame)
                total_rf_bytes += len(frame)

            if frames:
                rf.flush()

        now = time.time()

        if now - last_print >= 2.0:
            last_print = now

            if rtcm.counts:
                counts_text = ", ".join(
                    f"{msg}:{cnt}" for msg, cnt in sorted(rtcm.counts.items())
                )
                print(
                    f"[BASE RTCM] gps_bytes={total_gps_bytes} "
                    f"rf_bytes={total_rf_bytes} "
                    f"msg_counts={{ {counts_text} }}"
                )
            else:
                print(
                    f"[BASE RTCM] gps_bytes={total_gps_bytes} "
                    f"rf_bytes={total_rf_bytes} "
                    f"henuz RTCM D3 frame yakalanmadi"
                )


# ============================================================
# MAIN
# ============================================================

def main() -> None:
    print("[BASE] Baslatiliyor...")
    print(f"[BASE] GPS port: {GPS_PORT} @ {GPS_BAUD}")
    print(f"[BASE] RF  port: {RF_PORT} @ {RF_BAUD}")

    gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.05)
    rf = serial.Serial(RF_PORT, RF_BAUD, timeout=0.05)

    time.sleep(1.0)

    if CONFIGURE_BASE:
        configure_base(
            gps=gps,
            survey_seconds=SURVEY_SECONDS,
            survey_acc_m=SURVEY_ACC_M,
            ecef_x=ECEF_X,
            ecef_y=ECEF_Y,
            ecef_z=ECEF_Z,
        )

    # 8 dakika bekle, bu sırada RF'e veri gönderme
    wait_before_bridge(gps, WAIT_BEFORE_BRIDGE_SECONDS)

    # 8 dakika sonunda RTCM'i RF'e göndermeye başla
    bridge_base(gps, rf)


if __name__ == "__main__":
    main()
