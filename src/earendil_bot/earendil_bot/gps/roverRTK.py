#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# ROS 2 Humbe / Jazzy uyumlu RTK Rover Düğümüdür.
# RF Telemetri modülünden gelen RTCM paketlerini CRC-24Q kontrolü ile doğrular,
# geçerli paketleri GPS modülüne yazar ve NMEA verisinden /gps/fix (NavSatFix - Enlem, Boylam, Rakım) yayınlar.

import serial
import time
import threading
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from earendil_bot.gps.gps_math import bearing_between_gps_deg, haversine, angle_error_deg

# ============================================================
# CRC-24Q VE RTCM PAKET AYRIŞTIRICI (newroverrtk.py)
# ============================================================

CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF


class RTCMExtractor:
    """
    RF hattından gelen veride 0xD3 RTCM başlangıç baytını bulur,
    paket uzunluğunu ve CRC-24Q algoritması ile paket doğruluğunu denetler.
    Sadece geçerli RTCM paketlerini döndürür.
    """

    def __init__(self):
        self.buf = bytearray()
        self.counts = Counter()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            i = self.buf.find(b"\xD3")

            if i < 0:
                if len(self.buf) > 4096:
                    self.buf.clear()
                break

            if i > 0:
                del self.buf[:i]

            if len(self.buf) < 3:
                break

            if self.buf[1] & 0xFC:
                del self.buf[0]
                continue

            length = ((self.buf[1] & 0x03) << 8) | self.buf[2]
            total = 3 + length + 3

            if total < 6 or total > 1030:
                del self.buf[0]
                continue

            if len(self.buf) < total:
                break

            frame = bytes(self.buf[:total])
            del self.buf[:total]

            expected = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
            actual = crc24q(frame[:-3])

            if expected != actual:
                # Bozuk CRC - Paketi atla
                continue

            payload = frame[3:3 + length]
            if len(payload) >= 2:
                msgid = (payload[0] << 4) | (payload[1] >> 4)
                self.counts[msgid] += 1

            frames.append(frame)

        return frames


# ============================================================
# NMEA CÜMLE AYRIŞTIRICILARI & YARDIMCI FONKSİYONLAR
# ============================================================

def nmea_checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def make_cmd(body: str) -> bytes:
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def safe_float(val: str) -> float:
    try:
        return float(val)
    except Exception:
        return 0.0


def nmea_latlon_to_decimal(val: str, hemi: str):
    if not val or not hemi:
        return None

    try:
        if hemi in ("N", "S"):
            deg = float(val[:2])
            minutes = float(val[2:])
        else:
            deg = float(val[:3])
            minutes = float(val[3:])
    except Exception:
        return None

    dec = deg + (minutes / 60.0)

    if hemi in ("S", "W"):
        dec = -dec

    return dec


def parse_gga(line: str):
    """
    $GNGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...
    """
    try:
        body = line[1:].split("*")[0]
        parts = body.split(",")

        if not parts[0].endswith("GGA"):
            return None

        utc = parts[1]
        lat = nmea_latlon_to_decimal(parts[2], parts[3])
        lon = nmea_latlon_to_decimal(parts[4], parts[5])
        quality = int(parts[6]) if parts[6] else 0
        sats = int(parts[7]) if parts[7] else 0
        hdop = safe_float(parts[8])
        alt = safe_float(parts[9])  # Metre cinsinden Ortometrik Rakım Değeri

        return {
            "utc": utc,
            "lat": lat,
            "lon": lon,
            "quality": quality,
            "sats": sats,
            "hdop": hdop,
            "alt": alt,
        }
    except Exception:
        return None


def parse_rmc(line: str):
    try:
        body = line[1:].split("*")[0]
        parts = body.split(",")

        if not parts[0].endswith("RMC"):
            return None

        utc = parts[1]
        status = parts[2]
        lat = nmea_latlon_to_decimal(parts[3], parts[4])
        lon = nmea_latlon_to_decimal(parts[5], parts[6])
        quality = 1 if status == "A" else 0

        return {
            "utc": utc,
            "lat": lat,
            "lon": lon,
            "quality": quality,
            "sats": 0,
            "hdop": 0.0,
            "alt": 0.0,
        }
    except Exception:
        return None


def quality_text(q: int) -> str:
    if q == 4:
        return "RTK (FIXED - Hassas)"
    if q == 5:
        return "RTK (FLOAT - Hassas Değil)"
    if q == 2:
        return "DGPS (Düzeltilmiş)"
    if q == 1:
        return "SPS (Standart GPS)"
    if q == 0:
        return "GEÇERSİZ (Fix Yok)"
    return f"Bilinmeyen (Q{q})"


def make_map_links(lat: float, lon: float):
    osm_link = f"https://www.openstreetmap.org/?mlat={lat:.8f}&mlon={lon:.8f}#map=20/{lat:.8f}/{lon:.8f}"
    google_link = f"https://www.google.com/maps?q={lat:.8f},{lon:.8f}"
    return osm_link, google_link


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


# ============================================================
# ROS 2 RTK ROVER NODE (RoverRTKNode)
# ============================================================

class RoverRTKNode(Node):
    def __init__(self):
        super().__init__('roverRTK')

        # Parametreler
        self.declare_parameter('gps_port', '/dev/ttyUSB0')
        self.declare_parameter('radio_port', '/dev/ttyUSB1')
        self.declare_parameter('gps_baud', 115200)
        self.declare_parameter('radio_baud', 115200)
        self.declare_parameter('configure_rover', True)
        self.declare_parameter('min_pwm', 60)
        self.declare_parameter('max_pwm', 90)
        self.declare_parameter('target_pwm', 80)
        self.declare_parameter('cmd_vel_topic', '/earendil/control/command')
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)
        self.declare_parameter('enable_test_flow', False)
        self.declare_parameter('map_link_print_interval', 3.0)
        self.declare_parameter('map_link_topic', '/gps/map_link')
        self.declare_parameter('fallback_lat', 39.925000)
        self.declare_parameter('fallback_lon', 32.836000)

        self.gps_port = self.get_parameter('gps_port').value
        self.radio_port = self.get_parameter('radio_port').value
        self.gps_baud = self.get_parameter('gps_baud').value
        self.radio_baud = self.get_parameter('radio_baud').value
        self.config_rover = self.get_parameter('configure_rover').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.target_pwm = self.get_parameter('target_pwm').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_lat_param = self.get_parameter('target_lat').value
        self.target_lon_param = self.get_parameter('target_lon').value
        self.enable_test_flow = self.get_parameter('enable_test_flow').value
        self.map_link_print_interval = float(self.get_parameter('map_link_print_interval').value)
        self.map_link_topic = self.get_parameter('map_link_topic').value
        self.fallback_lat = float(self.get_parameter('fallback_lat').value)
        self.fallback_lon = float(self.get_parameter('fallback_lon').value)

        if "ttyUSBx" in self.gps_port:
            self.gps_port = "/dev/ttyUSB0"
        if "ttyUSBx" in self.radio_port:
            self.radio_port = "/dev/ttyUSB1"

        # Durum Değişkenleri
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_quality = 0
        self.current_sats = 0
        self.current_hdop = 0.0
        self.has_fix = False
        self.last_logged_quality = -1
        self.logging_muted = False

        self.stop_flag = threading.Event()

        # ROS2 Yayıncıları
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.map_link_pub = self.create_publisher(String, self.map_link_topic, 10)
        self.cmd_pub = self.create_publisher(String, self.cmd_vel_topic, 10)

        # Harita linki zamanlayıcısı (Her 3 saniyede bir yayınlar)
        self.create_timer(self.map_link_print_interval, self.publish_map_link_timer_cb)

        # Seri Bağlantıları Aç
        self.get_logger().info(f"[ROVER] Seri bağlantılar başlatılıyor...")
        self.get_logger().info(f"[ROVER] GPS Port: {self.gps_port} @ {self.gps_baud}")
        self.get_logger().info(f"[ROVER] RF Port: {self.radio_port} @ {self.radio_baud}")

        try:
            self.gps = serial.Serial(self.gps_port, self.gps_baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().error(f"GPS seri portu açılamadı {self.gps_port}: {e}")
            raise e

        try:
            self.rf = serial.Serial(self.radio_port, self.radio_baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().error(f"RF seri portu açılamadı {self.radio_port}: {e}")
            raise e

        time.sleep(1.0)

        if self.config_rover:
            self.configure_rover_gps()

        # İş Parçacıkları (Threads)
        self.t_rf_to_gps = threading.Thread(target=self.rf_to_gps_loop, daemon=True)
        self.t_gps_read = threading.Thread(target=self.gps_read_loop, daemon=True)

        self.t_rf_to_gps.start()
        self.t_gps_read.start()

        if self.enable_test_flow:
            self.get_logger().info("[ROVER] Otonom navigasyon test akışı başlatılıyor...")
            self.t_test_flow = threading.Thread(target=self.run_test_flow, daemon=True)
            self.t_test_flow.start()
        else:
            self.get_logger().info("[ROVER] Sadece GPS verisi (/gps/fix) ve Harita linki yayınlanıyor.")

    def publish_map_link_timer_cb(self):
        disp_lat = self.current_lat if self.current_lat is not None else self.fallback_lat
        disp_lon = self.current_lon if self.current_lon is not None else self.fallback_lon

        osm_link, google_link = make_map_links(disp_lat, disp_lon)
        link_msg = String()
        link_msg.data = google_link
        self.map_link_pub.publish(link_msg)

        if not self.logging_muted:
            if self.has_fix:
                self.get_logger().info(f"[GPS LİNK - CANLI RTK KONUM] Google Harita: {google_link}")
            else:
                self.get_logger().info(f"[GPS LİNK - FIX BEKLENİYOR] Google Harita: {google_link}")

    def configure_rover_gps(self) -> None:
        self.get_logger().info("[GPS KONFİG] LC29HEA alıcı ayarları Rover moduna alınıyor...")
        self.send_cmd("PQTMCFGRCVRMODE,W,0", wait=0.4)
        self.send_cmd("PQTMCFGRCVRMODE,W,1", wait=0.4)
        self.send_cmd("PQTMSAVEPAR", 0.5)
        self.send_cmd("PAIR062,2,0")  # GSA off
        self.send_cmd("PAIR062,3,0")  # GSV off
        self.send_cmd("PAIR062,5,0")  # VTG off
        self.send_cmd("PQTMCFGNMEADP,W,3,6,3,2,3,2")
        self.send_cmd("PAIR050,200")  # 5 Hz
        self.send_cmd("PAIR062,0,1")  # GGA on
        self.send_cmd("PAIR062,4,1")  # RMC on
        self.send_cmd("PQTMSAVEPAR", 1.0)
        self.get_logger().info("[GPS KONFİG] Rover ayarları başarıyla kaydedildi.")

    def send_cmd(self, body: str, wait: float = 0.3) -> None:
        cmd = make_cmd(body)
        self.get_logger().info(f"[GPS CMD] Send -> {cmd.decode().strip()}")
        self.gps.write(cmd)
        self.gps.flush()
        time.sleep(wait)
        try:
            while self.gps.in_waiting:
                resp = self.gps.readline().decode("ascii", errors="ignore").strip()
                if resp:
                    self.get_logger().info(f"   [GPS YANIT] <- {resp}")
        except Exception:
            pass

    def rf_to_gps_loop(self) -> None:
        """
        RF Telemetri modülünden gelen RTCM verisini CRC-24Q kontrolü ile doğrular.
        Sadece geçerli RTCM paketlerini GPS alıcısına iletir.
        """
        rtcm = RTCMExtractor()
        total_rf_bytes = 0
        total_valid_rtcm_bytes = 0
        last_print = time.time()

        while not self.stop_flag.is_set() and rclpy.ok():
            try:
                data = self.rf.read(4096)
                if data:
                    total_rf_bytes += len(data)
                    frames = rtcm.feed(data)
                    for f in frames:
                        self.gps.write(f)
                        total_valid_rtcm_bytes += len(f)
                    if frames:
                        self.gps.flush()
            except Exception as e:
                self.get_logger().error(f"[RTCM HATA] RF -> GPS RTCM köprü hatası: {e}")
                time.sleep(0.5)
                continue

            now = time.time()
            if not self.logging_muted and (now - last_print >= 5.0):
                last_print = now
                if rtcm.counts:
                    counts_text = ", ".join(
                        f"Tip {msg}: {cnt} adet" for msg, cnt in sorted(rtcm.counts.items())
                    )
                    self.get_logger().info(
                        f"[RTCM ALICI (CRC24Q)] RF Bayt={total_rf_bytes} | Geçerli RTCM Bayt={total_valid_rtcm_bytes} | Düzeltmeler: {counts_text}"
                    )
                else:
                    self.get_logger().info(
                        f"[RTCM ALICI (CRC24Q)] RF Bayt={total_rf_bytes} | Geçerli RTCM Bayt={total_valid_rtcm_bytes} | Henüz doğrulunmış RTCM paketi alınamadı."
                    )

    def gps_read_loop(self) -> None:
        """
        GPS alıcısından gelen NMEA satırlarını okur.
        $GNGGA ve $GNRMC verilerini ayrıştırarak /gps/fix (NavSatFix) olarak yayınlar.
        """
        nmea = NMEALineParser()
        last_map_print = 0.0
        last_read_time = time.time()
        last_warn_time = 0.0

        while not self.stop_flag.is_set() and rclpy.ok():
            try:
                data = self.gps.read(4096)
                now = time.time()
                if not data:
                    if (now - last_read_time > 5.0) and (now - last_warn_time > 5.0):
                        last_warn_time = now
                        self.get_logger().warning(
                            f"[GPS UYARI] {self.gps_port} portundan veri okunamadı. Bağlantıyı kontrol edin."
                        )
                    continue
                last_read_time = now

                lines = nmea.feed(data)
            except Exception as e:
                self.get_logger().error(f"[GPS HATA] Okuma hatası: {e}")
                time.sleep(0.5)
                continue

            for line in lines:
                gga_or_rmc = None
                if "GGA" in line:
                    gga_or_rmc = parse_gga(line)
                elif "RMC" in line:
                    gga_or_rmc = parse_rmc(line)

                if gga_or_rmc is not None:
                    q = gga_or_rmc["quality"]
                    lat = gga_or_rmc["lat"]
                    lon = gga_or_rmc["lon"]
                    alt = gga_or_rmc["alt"]
                    sats = gga_or_rmc["sats"]
                    hdop = gga_or_rmc["hdop"]

                    if lat is not None and lon is not None:
                        self.current_lat = lat
                        self.current_lon = lon
                        self.current_alt = alt
                        self.current_quality = q
                        self.current_sats = sats
                        self.current_hdop = hdop
                        self.has_fix = True

                        # NavSatFix Mesajı Yayınlama (Enlem, Boylam, Rakım)
                        fix_msg = NavSatFix()
                        fix_msg.header.stamp = self.get_clock().now().to_msg()
                        fix_msg.header.frame_id = 'gps'
                        fix_msg.latitude = lat
                        fix_msg.longitude = lon
                        fix_msg.altitude = float(alt) if alt is not None else 0.0

                        if q == 4:
                            fix_msg.status.status = 2  # STATUS_GBAS_FIX (RTK FIXED)
                        elif q == 5:
                            fix_msg.status.status = 2  # RTK FLOAT
                        elif q == 2:
                            fix_msg.status.status = 1  # DGPS
                        elif q == 1:
                            fix_msg.status.status = 0  # SPS FIX
                        else:
                            fix_msg.status.status = -1  # NO FIX

                        self.gps_pub.publish(fix_msg)

                    disp_lat = lat if lat is not None else (self.current_lat if self.current_lat is not None else self.fallback_lat)
                    disp_lon = lon if lon is not None else (self.current_lon if self.current_lon is not None else self.fallback_lon)

                    osm_link, google_link = make_map_links(disp_lat, disp_lon)
                    link_msg = String()
                    link_msg.data = google_link
                    self.map_link_pub.publish(link_msg)

                    now = time.time()
                    quality_changed = (q != self.last_logged_quality)
                    if not self.logging_muted and (quality_changed or (now - last_map_print >= self.map_link_print_interval)):
                        last_map_print = now
                        self.last_logged_quality = q
                        if lat is not None and lon is not None:
                            self.get_logger().info(
                                f"[GPS VERİSİ] Kalite: {quality_text(q)} | Enlem: {lat:.8f}, Boylam: {lon:.8f} | Yükseklik/Rakım: {alt}m | Uydu: {sats} | HDOP: {hdop}"
                            )
                            self.get_logger().info(f"[GPS LİNK - CANLI KONUM] Google Harita: {google_link}")

    def run_test_flow(self):
        try:
            self.get_logger().info("[TEST] Test iş parçacığı başlatıldı. GPS Fix bekleniyor...")
            while rclpy.ok() and not self.has_fix:
                time.sleep(0.5)

            if not rclpy.ok():
                return

            self.get_logger().info("[TEST] GPS Sinyali alındı.")
        except Exception as e:
            self.get_logger().error(f"[TEST HATA] Test akışı hatası: {e}")

    def stop_node(self):
        self.stop_flag.set()
        try:
            if hasattr(self, 'rf') and self.rf and self.rf.is_open:
                self.rf.close()
            if hasattr(self, 'gps') and self.gps and self.gps.is_open:
                self.gps.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RoverRTKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()