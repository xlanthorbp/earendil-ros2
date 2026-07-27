# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import serial
import time
import threading
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from earendil_bot.gps.gps_math import bearing_between_gps_deg, haversine, angle_error_deg


# ============================================================
# ROVER AYARLARI (ROS2 Node Parameters will default to these)
# ============================================================

MAP_LINK_PRINT_INTERVAL = 1.0  # saniye


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


class NMEALineParser:
    """
    GPS stream içinden sadece '$' ile başlayan NMEA/PQTM satırlarını toplar.
    """

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
    RF'ten gelen RTCM frame'lerini saymak için kullanılıyor.
    Gelen data yine de GPS'e ham olarak basılıyor.
    """

    def __init__(self):
        self.buf = bytearray()
        self.counts = Counter()

    def feed(self, data: bytes):
        self.buf.extend(data)

        while True:
            idx = self.buf.find(b"\xD3")

            if idx < 0:
                if len(self.buf) > 4096:
                    self.buf.clear()
                return

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 3:
                return

            length = ((self.buf[1] & 0x03) << 8) | self.buf[2]

            if length > 1023:
                del self.buf[0]
                continue

            total_len = 3 + length + 3

            if len(self.buf) < total_len:
                return

            frame = bytes(self.buf[:total_len])
            del self.buf[:total_len]

            if length >= 2:
                payload = frame[3:3 + length]
                msg_id = (payload[0] << 4) | (payload[1] >> 4)
                self.counts[msg_id] += 1


def safe_float(s, default=None):
    try:
        if s == "":
            return default
        return float(s)
    except Exception:
        return default


def nmea_latlon_to_decimal(value: str, hemi: str):
    """
    NMEA formatı:
    Latitude:  ddmm.mmmmm
    Longitude: dddmm.mmmmm
    """

    if not value or not hemi:
        return None

    raw = float(value)
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    dec = degrees + minutes / 60.0

    if hemi in ("S", "W"):
        dec = -dec

    return dec


def parse_gga(line: str):
    """
    $GNGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...

    quality:
      0 = no fix
      1 = SPS / normal GPS
      2 = DGPS
      4 = RTK FIXED
      5 = RTK FLOAT
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
        alt = safe_float(parts[9])

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
    """
    $GNRMC,time,status,lat,N,lon,E,speed,track,date,...
    status: 'A' = Valid, 'V' = Invalid
    """
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
    osm_link = (
        f"https://www.openstreetmap.org/"
        f"?mlat={lat:.8f}&mlon={lon:.8f}"
        f"#map=20/{lat:.8f}/{lon:.8f}"
    )

    google_link = f"https://www.google.com/maps?q={lat:.8f},{lon:.8f}"

    return osm_link, google_link


# ============================================================
# ROS2 RTK ROVER NODE
# ============================================================

class RoverRTKNode(Node):
    def __init__(self):
        super().__init__('roverRTK')

        # Declare ROS Parameters with sensible defaults matching hardware_params.yaml
        self.declare_parameter('gps_port', '/dev/ttyUSB0')
        self.declare_parameter('radio_port', '/dev/ttyUSB1')
        self.declare_parameter('gps_baud', 460800)
        self.declare_parameter('radio_baud', 115200)
        self.declare_parameter('configure_rover', True)
        self.declare_parameter('min_pwm', 60)
        self.declare_parameter('max_pwm', 90)
        self.declare_parameter('target_pwm', 80)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
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

        # Fallbacks for hardware_params.yaml default dummy placeholders
        if "ttyUSBx" in self.gps_port:
            self.gps_port = "/dev/ttyUSB0"
        if "ttyUSBx" in self.radio_port:
            self.radio_port = "/dev/ttyUSB1"

        # State Variables
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

        # ROS Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.map_link_pub = self.create_publisher(String, self.map_link_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer to ALWAYS publish and print Google Maps link every 3 seconds
        self.create_timer(self.map_link_print_interval, self.publish_map_link_timer_cb)

        # Serial Connections
        self.get_logger().info(f"[ROVER] Starting serial connections...")
        self.get_logger().info(f"[ROVER] GPS Port: {self.gps_port} @ {self.gps_baud}")
        self.get_logger().info(f"[ROVER] RF Port: {self.radio_port} @ {self.radio_baud}")

        try:
            self.gps = serial.Serial(self.gps_port, self.gps_baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open GPS serial port {self.gps_port}: {e}")
            raise e

        try:
            self.rf = serial.Serial(self.radio_port, self.radio_baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open RF serial port {self.radio_port}: {e}")
            raise e

        time.sleep(1.0)

        if self.config_rover:
            self.configure_rover_gps()

        # Threads
        self.t_rf_to_gps = threading.Thread(
            target=self.rf_to_gps_loop,
            daemon=True
        )
        self.t_gps_read = threading.Thread(
            target=self.gps_read_loop,
            daemon=True
        )

        self.t_rf_to_gps.start()
        self.t_gps_read.start()

        # Background test sequence thread (only if enabled)
        if self.enable_test_flow:
            self.get_logger().info("[ROVER] Otonom navigasyon test akışı başlatılıyor...")
            self.t_test_flow = threading.Thread(
                target=self.run_test_flow,
                daemon=True
            )
            self.t_test_flow.start()
        else:
            self.get_logger().info("[ROVER] Otonom navigasyon testi kapalı. Sadece GPS verisi ve Harita linki yayınlanıyor.")

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
        self.get_logger().info("[GPS KONFİG] LC29HEA alıcı ayarları Rover (Gezgin) moduna alınıyor...")
        # 1. Modu sıfırlamak için önce Standby/Normal (0), ardından Rover (1) moduna al
        self.send_cmd("PQTMCFGRCVRMODE,W,0", wait=0.4)
        self.send_cmd("PQTMCFGRCVRMODE,W,1", wait=0.4)
        self.send_cmd("PQTMSAVEPAR", 0.5)
        # 2. Gereksiz NMEA cümlelerini kapat, GGA ve RMC'yi aç
        self.send_cmd("PAIR062,2,0")  # GSA off
        self.send_cmd("PAIR062,3,0")  # GSV off
        self.send_cmd("PAIR062,5,0")  # VTG off
        self.send_cmd("PQTMCFGNMEADP,W,3,6,3,2,3,2")
        self.send_cmd("PAIR050,200")  # 5 Hz
        self.send_cmd("PAIR062,0,1")   # GGA on
        self.send_cmd("PAIR062,4,1")   # RMC on
        self.send_cmd("PQTMSAVEPAR", 1.0)
        self.get_logger().info("[GPS KONFİG] Rover modu ayarları başarıyla gönderildi ve Flash hafızaya kaydedildi.")

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
        rtcm = RTCMExtractor()
        total_rf_bytes = 0
        total_gps_write_bytes = 0
        last_print = time.time()

        while not self.stop_flag.is_set() and rclpy.ok():
            try:
                data = self.rf.read(4096)
                if data:
                    total_rf_bytes += len(data)
                    self.gps.write(data)
                    self.gps.flush()
                    total_gps_write_bytes += len(data)
                    rtcm.feed(data)
            except Exception as e:
                self.get_logger().error(f"[RTCM HATA] RF -> GPS RTCM köprü hatası: {e}")
                time.sleep(0.5)
                continue

            now = time.time()
            if not self.logging_muted and (now - last_print >= 5.0):  # Throttled to 5 seconds to reduce terminal clutter
                last_print = now
                if rtcm.counts:
                    counts_text = ", ".join(
                        f"Tip {msg}: {cnt} adet" for msg, cnt in sorted(rtcm.counts.items())
                    )
                    self.get_logger().info(
                        f"[RTCM ALICI] RF Bayt={total_rf_bytes} | GPS Yazılan={total_gps_write_bytes} | Düzeltmeler: {counts_text}"
                    )
                else:
                    self.get_logger().info(
                        f"[RTCM ALICI] RF Bayt={total_rf_bytes} | GPS Yazılan={total_gps_write_bytes} | Henüz RTCM düzeltme paketi alınamadı."
                    )

    def gps_read_loop(self) -> None:
        nmea = NMEALineParser()
        last_map_print = 0.0
        last_read_time = time.time()
        last_warn_time = 0.0
        last_raw_debug = 0.0

        while not self.stop_flag.is_set() and rclpy.ok():
            try:
                data = self.gps.read(4096)
                now = time.time()
                if not data:
                    if (now - last_read_time > 5.0) and (now - last_warn_time > 5.0):
                        last_warn_time = now
                        self.get_logger().warning(
                            f"[GPS UYARI] {self.gps_port} ({self.gps_baud} baud) portundan veri okunamadı. USB kablosunu kontrol edin."
                        )
                    continue
                last_read_time = now

                lines = nmea.feed(data)
                if not lines and (now - last_raw_debug > 5.0):
                    last_raw_debug = now
                    self.get_logger().info(
                        f"[GPS HAM TEŞHİS] {self.gps_port} portundan {len(data)} bayt okundu. Ham veri örneği: {data[:50]}"
                    )
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

                        # Publish NavSatFix message
                        fix_msg = NavSatFix()
                        fix_msg.header.stamp = self.get_clock().now().to_msg()
                        fix_msg.header.frame_id = 'gps'
                        fix_msg.latitude = lat
                        fix_msg.longitude = lon
                        fix_msg.altitude = alt if alt is not None else 0.0

                        if q == 4:
                            fix_msg.status.status = 2  # STATUS_GBAS_FIX
                        elif q == 5:
                            fix_msg.status.status = 2  # RTK FLOAT mapped as GBAS_FIX
                        elif q == 2:
                            fix_msg.status.status = 1  # STATUS_SBAS_FIX
                        elif q == 1:
                            fix_msg.status.status = 0  # STATUS_FIX
                        else:
                            fix_msg.status.status = -1 # STATUS_NO_FIX

                        self.gps_pub.publish(fix_msg)

                    # Determine display coordinates (real GPS position if available, else fallback/last known)
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
                                f"[GPS VERİSİ] Kalite: {quality_text(q)} | Enlem: {lat:.8f}, Boylam: {lon:.8f} | Yükseklik: {alt}m | Uydu: {sats} | HDOP: {hdop}"
                            )
                            self.get_logger().info(f"[GPS LİNK - CANLI KONUM] Google Harita: {google_link}")
                            self.get_logger().info(f"[GPS LİNK - CANLI KONUM] OpenStreetMap: {osm_link}")
                        else:
                            status_tag = "SON BİLİNEN KONUM" if self.current_lat is not None else "FIX BEKLENİYOR - ÖRNEK KONUM"
                            self.get_logger().info(
                                f"[GPS VERİSİ] Kalite: {quality_text(q)} | Sinyal zayıf (Açık alana çıkarın) | Uydu: {sats} | HDOP: {hdop}"
                            )
                            self.get_logger().info(f"[GPS LİNK - {status_tag}] Google Harita: {google_link}")
                elif line.startswith("$PQTM"):
                    pass

    def pwm_to_velocity(self, target_pwm):
        if self.max_pwm == self.min_pwm:
            return 1.0
        vel = (target_pwm - self.min_pwm) / (self.max_pwm - self.min_pwm)
        return max(0.0, min(1.0, vel))

    def send_velocity(self, linear_x, angular_z, duration_sec):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < duration_sec):
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)

        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        time.sleep(0.1)
        self.cmd_pub.publish(stop_cmd)

    def run_test_flow(self):
        try:
            self.get_logger().info("[TEST] Test iş parçacığı başlatıldı. GPS Fix alınması bekleniyor...")
            while rclpy.ok() and not self.has_fix:
                time.sleep(0.5)

            if not rclpy.ok():
                return

            # ADIM 1: 5 saniye beklenip bu sürede gelen en iyi GPS koordinatını bul
            self.get_logger().info("[TEST] GPS Sinyali tespit edildi. 5 saniye bekleniyor ve bu süredeki EN İYİ GPS koordinatı belirleniyor...")
            
            best_lat = None
            best_lon = None
            best_quality = -1
            best_hdop = 999.0
            
            start_wait = time.time()
            while rclpy.ok() and (time.time() - start_wait < 5.0):
                if self.current_lat is not None and self.current_lon is not None:
                    is_better = False
                    if best_lat is None:
                        is_better = True
                    elif self.current_quality > best_quality:
                        is_better = True
                    elif self.current_quality == best_quality and self.current_hdop < best_hdop:
                        is_better = True
                        
                    if is_better:
                        best_lat = self.current_lat
                        best_lon = self.current_lon
                        best_quality = self.current_quality
                        best_hdop = self.current_hdop
                time.sleep(0.1)
                
            if best_lat is not None:
                lat_start = best_lat
                lon_start = best_lon
            else:
                lat_start = self.current_lat
                lon_start = self.current_lon
                best_quality = self.current_quality
                best_hdop = self.current_hdop

            _, gmaps_start = make_map_links(lat_start, lon_start)
            self.get_logger().info(
                f"[TEST] Belirlenen En İyi Başlangıç Konumu (P_start): ({lat_start:.8f}, {lon_start:.8f}) "
                f"| Kalite: {quality_text(best_quality)} | HDOP: {best_hdop}"
            )
            self.get_logger().info(f"[TEST LİNK] P_start Google Harita: {gmaps_start}")

            # ADIM 2: 80 PWM ile 5 Saniye İleri Sür (Yön Vektörü Oluşturmak İçin)
            forward_duration = 5.0
            v_fwd = self.pwm_to_velocity(self.target_pwm)

            self.get_logger().info(f"[TEST] {self.target_pwm} PWM hızla (hız={v_fwd:.4f}) {forward_duration:.1f} saniye ileri sürülüyor...")
            self.send_velocity(v_fwd, 0.0, forward_duration)
            self.get_logger().info("[TEST] İleri sürüş bitti. GPS konumunun oturması için 3 saniye bekleniyor...")
            time.sleep(3.0)

            # ADIM 3: Bitiş GPS Konumunu Al (P_end)
            self.get_logger().info("[TEST] Bitiş konumu 2 saniye boyunca ortalanıyor...")
            lat_sum = 0.0
            lon_sum = 0.0
            count = 0
            for _ in range(10):
                if self.current_lat is not None and self.current_lon is not None:
                    lat_sum += self.current_lat
                    lon_sum += self.current_lon
                    count += 1
                time.sleep(0.2)

            if count > 0:
                lat_end = lat_sum / count
                lon_end = lon_sum / count
            else:
                lat_end = self.current_lat
                lon_end = self.current_lon

            _, gmaps_end = make_map_links(lat_end, lon_end)
            self.get_logger().info(f"[TEST] Bitiş Koordinatı (P_end): ({lat_end:.8f}, {lon_end:.8f})")
            self.get_logger().info(f"[TEST LİNK] P_end Google Harita: {gmaps_end}")

            # ADIM 4: Yön Vektörünü Hesapla
            theta_start = bearing_between_gps_deg(lat_start, lon_start, lat_end, lon_end)
            self.get_logger().info(f"[TEST] =========================================")
            self.get_logger().info(f"[TEST] Hesaplanan Araç Yönü: {theta_start:.2f}°")
            self.get_logger().info(f"[TEST] =========================================")

            # ADIM 5: Hedef Koordinat Girişi İste
            # ADIM 5: Hedef Koordinat Girişi İste / Parametre Kontrolü
            if self.target_lat_param != 0.0 and self.target_lon_param != 0.0:
                target_lat = self.target_lat_param
                target_lon = self.target_lon_param
                self.get_logger().info(f"[TEST] Parametrelerden alınan hedef koordinat kullanılıyor: ({target_lat:.8f}, {target_lon:.8f})")
            else:
                self.logging_muted = True
                try:
                    if not sys.stdin.isatty():
                        self.get_logger().warning("[TEST] İnteraktif terminal bulunamadı (non-interactive shell). Manuel hedef koordinat girişi atlanıyor.")
                        return
                    while rclpy.ok():
                        print("\n" + "="*50)
                        print(" MANUEL HEDEF KOORDİNAT GİRİŞİ ")
                        print("="*50)
                        print(f"Mevcut RTK Kalitesi: {quality_text(self.current_quality)} ({self.current_sats} uydu)")
                        print(f"Mevcut Konum: ({lat_end:.8f}, {lon_end:.8f})")
                        print("Lütfen enlem ve boylamı aralarında virgül olacak şekilde girin.")
                        print("Örnek: 39.925000, 32.836000")
                        print("="*50)

                        try:
                            user_input = input("Hedef Koordinat (lat, lon): ")
                            if not user_input.strip():
                                continue
                            lat_str, lon_str = user_input.split(",")
                            target_lat = float(lat_str.strip())
                            target_lon = float(lon_str.strip())
                            break
                        except KeyboardInterrupt:
                            return
                        except Exception as e:
                            print(f"Koordinat ayrıştırma hatası: {e}. Lütfen tekrar deneyin.")
                except EOFError:
                    self.get_logger().warning("[TEST] EOFError: Terminal girdisi alınamadı. Manuel giriş atlanıyor.")
                    return
                finally:
                    self.logging_muted = False

            # ADIM 6: Dönüş Açısı ve Süresini Hesapla
            theta_target = bearing_between_gps_deg(lat_end, lon_end, target_lat, target_lon)
            distance = haversine(lat_end, lon_end, target_lat, target_lon)
            delta_theta = angle_error_deg(theta_target, theta_start)

            _, gmaps_target = make_map_links(target_lat, target_lon)
            self.get_logger().info(f"[TEST] Hedef Koordinat: ({target_lat:.8f}, {target_lon:.8f})")
            self.get_logger().info(f"[TEST LİNK] Hedef Google Harita: {gmaps_target}")
            self.get_logger().info(f"[TEST] Hedefe Olan Mesafe: {distance:.2f} metre")
            self.get_logger().info(f"[TEST] Hedef Açısı (Bearing): {theta_target:.2f}°")
            self.get_logger().info(f"[TEST] Gerekli Dönüş Açısı: {delta_theta:.2f}°")

            # Turning speed at 80 PWM: 130 degrees in 5 seconds = 26.0 deg/s
            turn_duration = abs(delta_theta) / 26.0
            w_turn = self.pwm_to_velocity(self.target_pwm)

            # Positive delta_theta means target is clockwise (right).
            # Negative delta_theta means target is counter-clockwise (left).
            # In ROS, positive angular.z turns left, negative turns right.
            if delta_theta > 0:
                angular_z = -w_turn  # Turn right (sağa)
                direction_str = "SAĞA"
            else:
                angular_z = w_turn   # Turn left (sola)
                direction_str = "SOLA"

            self.get_logger().info(f"[TEST] {self.target_pwm} PWM hızla (w={angular_z:.4f}) {direction_str} {abs(delta_theta):.2f}° dönülüyor ({turn_duration:.2f} saniye)...")
            self.send_velocity(0.0, angular_z, turn_duration)
            self.get_logger().info("[TEST] Dönüş tamamlandı. 1 saniye bekleniyor...")
            time.sleep(1.0)

            # STEP 7: Drive Straight towards Target
            drive_duration = distance / 0.14
            self.get_logger().info(f"[TEST] Hedefe doğru {distance:.2f} metre dümdüz sürülüyor ({drive_duration:.2f} saniye)...")
            self.send_velocity(v_fwd, 0.0, drive_duration)

            if self.current_lat is not None and self.current_lon is not None:
                _, gmaps_final = make_map_links(self.current_lat, self.current_lon)
                self.get_logger().info(f"[TEST LİNK] Ulaşılan Son Konum Google Harita: {gmaps_final}")

            self.get_logger().info("[TEST] Hedefe ulaşıldı! Navigasyon testi başarıyla tamamlandı.")
        except Exception as e:
            self.get_logger().error(f"[TEST HATA] Test akışında beklenmedik hata: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


# ============================================================
# MAIN
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    print("[ROVER] Starting roverRTK...")
    node = RoverRTKNode()

    # MultiThreadedExecutor is needed because run_test_flow is blocking/running in a thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_flag.set()
        try:
            node.get_logger().info("[ROVER] Shutting down...")
            stop_cmd = Twist()
            node.cmd_pub.publish(stop_cmd)
        except Exception:
            pass
        
        try:
            node.destroy_node()
        except Exception:
            pass
            
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
