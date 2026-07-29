#!/usr/bin/env python3
"""
Earendil Bot - Hassas Otonom GPS Navigasyon Düğümü (gps_navigator_node.py)
-------------------------------------------------------------------------
Bu düğüm, ARC'26 Otonom Görev (Stage 1, 2, 3, 4) ihtiyaçlarına tam uyumlu olarak 
gelen hedef GPS koordinatlarına robotu hassas ve güvenli bir şekilde götürür.

Özellikler:
1. Stage 1-4 dinamik parametre uyarlaması (Arama Alanı, Tünel Girişi, Airlock hassas varış yarıçapları).
2. Çift fazlı sürüş mantığı: Yerinde Dönüş (Rotate Phase) ➔ İleri Sürüş ve Şerit Takibi (Drive Phase).
3. Yumuşak hız profili (Yaklaşırken yavaşlama, ivmelenme/yavaşlama rampa kontrolü).
4. RSCP Entegrasyonu (Gelen komutla otomatik başlama, varışta /rscp/feedback/task_finished tetikleme).
5. Donanımsal Watchdog ve Disarm Güvenliği (Veri kesintisinde veya Disarm durumunda anında durma).
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float64, String, Bool, Int32, Empty
from geometry_msgs.msg import Twist

from earendil_autonomy.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class GPSNavigatorNode(Node):
    def __init__(self):
        super().__init__('gps_navigator_node')

        # ---------------------------------------------------------
        # ROS 2 Parametreleri ve Varsayılan Ayarlar
        # ---------------------------------------------------------
        self.declare_parameter('heading_tolerance_deg', 6.0)     # Yönlenme toleransı (Derece)
        self.declare_parameter('default_arrival_radius', 0.6)   # Varsayılan varış yarıçapı (Metre)
        self.declare_parameter('max_linear_x', 0.5)             # Maksimum ileri lineer hız (m/s)
        self.declare_parameter('max_angular_z', 0.8)            # Maksimum açısal dönüş hızı (rad/s)
        self.declare_parameter('min_pwm', 60)                   # Motor sürücü alt PWM limiti
        self.declare_parameter('max_pwm', 100)                  # Motor sürücü üst PWM limiti
        self.declare_parameter('kp_angular', 2.2)               # Yerinde dönüş P-kazancı
        self.declare_parameter('kp_lane', 1.4)                  # İleri sürüş şerit takip P-kazancı
        self.declare_parameter('dry_run', False)                # Test modu (Motor çalıştırmaz)

        self.heading_tol = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.arrival_radius = float(self.get_parameter('default_arrival_radius').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.kp_lane = float(self.get_parameter('kp_lane').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        # ---------------------------------------------------------
        # Durum Değişkenleri (State Variables)
        # ---------------------------------------------------------
        self.current_stage = 0        # 1, 2, 3, 4
        self.is_armed = False         # Otonom aktiflik
        self.nav_state = 'IDLE'       # IDLE, ROTATING, DRIVING, ARRIVED, PAUSED

        self.current_lat = None
        self.current_lon = None
        self.current_alt = 0.0
        self.mag_heading = None

        self.target_lat = None
        self.target_lon = None

        self.last_gps_time = 0.0
        self.last_mag_time = 0.0
        self.aligned = False

        # ---------------------------------------------------------
        # Abonelikler (Subscribers)
        # ---------------------------------------------------------
        # Sensör Verileri (RTK GPS & STM32 H7 Heading)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.heading_cb, 10)

        # Sistem Komutları ve Goal Topic'leri
        self.create_subscription(Int32, '/rscp/command/set_stage', self.stage_cb, 10)
        self.create_subscription(Bool, '/rscp/command/arm', self.arm_cb, 10)
        self.create_subscription(NavSatFix, '/navigator/goal', self.direct_goal_cb, 10)
        self.create_subscription(Empty, '/navigator/cancel', self.cancel_cb, 10)

        # ---------------------------------------------------------
        # Yayıncılar (Publishers)
        # ---------------------------------------------------------
        # STM32 H7 Motor Komutları ("f40", "l50", "stop")
        self.pub_h7_cmd = self.create_publisher(String, '/earendil/control/command', 10)

        # Durum ve Varış Bildirimleri
        self.pub_nav_status = self.create_publisher(String, '/navigator/status', 10)
        self.pub_arrived = self.create_publisher(Bool, '/navigator/arrived', 10)

        # 10 Hz Ana Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🧭 GPS Navigator Node aktif ve hedef komutları bekliyor.")

    # ---------------------------------------------------------
    # Callback Fonksiyonları
    # ---------------------------------------------------------
    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        self.last_gps_time = time.time()

    def heading_cb(self, msg: Float64):
        # Deg (0-360) ➔ Radian
        self.mag_heading = math.radians(float(msg.data))
        self.last_mag_time = time.time()

    def stage_cb(self, msg: Int32):
        self.current_stage = msg.data
        # Aşamaya göre varış toleransını dinamik olarak ayarla
        if self.current_stage in (1, 2):
            self.arrival_radius = 1.0  # Arama alanı merkez noktaları için esnek tolerans
        elif self.current_stage == 3:
            self.arrival_radius = 0.6  # Tünel girişi için hassas yaklaşma
        elif self.current_stage == 4:
            self.arrival_radius = 0.5  # Airlock üs dönüşü için tam hassasiyet
        
        self.get_logger().info(f"Stage {self.current_stage} ayarlandı. Varış yarıçapı: {self.arrival_radius:.2f}m")

    def arm_cb(self, msg: Bool):
        self.is_armed = msg.data
        if not self.is_armed:
            self.get_logger().warn("🛑 Robot DISARM edildi. Navigasyon durduruluyor.")
            self.stop_robot()
            self.nav_state = 'PAUSED'
        else:
            self.get_logger().info("🟢 Robot ARM edildi. Otonom navigasyon etkin.")
            if self.target_lat is not None and self.target_lon is not None and self.nav_state != 'ARRIVED':
                self.nav_state = 'ROTATING'

    def direct_goal_cb(self, msg: NavSatFix):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.check_and_start_goal()

    def cancel_cb(self, msg: Empty):
        self.get_logger().info("⚠️ Navigasyon görevi iptal edildi.")
        self.target_lat = None
        self.target_lon = None
        self.stop_robot()
        self.nav_state = 'IDLE'

    def check_and_start_goal(self):
        if self.target_lat is not None and self.target_lon is not None:
            self.aligned = False
            self.nav_state = 'ROTATING'
            self.get_logger().info(
                f"📍 Yeni Hedef Alındı: ({self.target_lat:.6f}, {self.target_lon:.6f}) - Stage: {self.current_stage}"
            )

    # ---------------------------------------------------------
    # Motor Komut Üretici (H7 Command)
    # ---------------------------------------------------------
    def send_motor_cmd(self, v: float, w: float):
        """Lineer (v) ve Açısal (w) hızları STM32 H7 komut formatına çevirir."""
        if self.dry_run or not self.is_armed:
            cmd_str = "stop"
        else:
            if abs(w) > 0.08:
                # Açısal dönüş komutu (l = sola dön, r = sağa dön)
                ratio = min(1.0, abs(w) / self.max_angular_z)
                pwm = self.min_pwm + int(ratio * (self.max_pwm - self.min_pwm))
                pwm = max(self.min_pwm, min(self.max_pwm, pwm))
                cmd_str = f"l{pwm}" if w > 0 else f"r{pwm}"
            elif abs(v) > 0.03:
                # Lineer hareket komutu (f = ileri, b = geri)
                ratio = min(1.0, abs(v) / self.max_linear_x)
                pwm = self.min_pwm + int(ratio * (self.max_pwm - self.min_pwm))
                pwm = max(self.min_pwm, min(self.max_pwm, pwm))
                cmd_str = f"f{pwm}" if v > 0 else f"b{pwm}"
            else:
                cmd_str = "stop"

        msg = String()
        msg.data = cmd_str
        self.pub_h7_cmd.publish(msg)

    def stop_robot(self):
        self.send_motor_cmd(0.0, 0.0)

    # ---------------------------------------------------------
    # Ana Kontrol Döngüsü (10 Hz)
    # ---------------------------------------------------------
    def control_loop(self):
        # Durum Yayını
        status_msg = String()
        status_msg.data = self.nav_state
        self.pub_nav_status.publish(status_msg)

        # Hedef kontrolü
        if self.target_lat is None or self.target_lon is None or self.nav_state in ('IDLE', 'ARRIVED', 'PAUSED'):
            return

        # Donanım & Sensör Watchdog Kontrolleri
        now = time.time()
        if self.mag_heading is None or (now - self.last_mag_time > 1.5):
            self.get_logger().warn("⚠️ Manyetometre verisi kesildi veya eski! Robot durduruluyor.", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        if self.current_lat is None or (now - self.last_gps_time > 2.0):
            self.get_logger().warn("⚠️ GPS verisi kesildi veya eski! Robot durduruluyor.", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # Mesafe ve Hedef Açı (Target Bearing) Hesabı
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        distance = haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)

        # Hedefe Varış Kontrolü
        if distance <= self.arrival_radius:
            self.nav_state = 'ARRIVED'
            self.get_logger().info(f"🎯 HEDEFE VARILDI! Kalan Mesafe: {distance:.2f}m")
            self.stop_robot()

            # Geri Bildirim Yayınları (mission_manager_node /navigator/arrived dinleyerek TaskFinished basar)
            arrived_msg = Bool()
            arrived_msg.data = True
            self.pub_arrived.publish(arrived_msg)

            return

        # Açısal Hata (Heading Error in Radians)
        heading_error = angle_error_rad(target_bearing, self.mag_heading)

        self.get_logger().info(
            f"[{self.nav_state}] Mesafe: {distance:.2f}m | Hedef Açı: {math.degrees(target_bearing):.1f}° | "
            f"Pusula: {math.degrees(self.mag_heading):.1f}° | Hata: {math.degrees(heading_error):.1f}°",
            throttle_duration_sec=1.0
        )

        v = 0.0
        w = 0.0

        # FAZ 1: Yerinde Dönüş (ROTATING Phase)
        if not self.aligned:
            self.nav_state = 'ROTATING'
            if abs(heading_error) > self.heading_tol:
                # P-Kontrol ile yerinde dön
                w = self.kp_angular * heading_error
                w = max(-self.max_angular_z, min(self.max_angular_z, w))
                v = 0.0
            else:
                self.aligned = True
                self.nav_state = 'DRIVING'
                self.get_logger().info("✅ Hedef açıya hizalandı! İleri sürüş fazına geçiliyor.")

        # FAZ 2: İleri Sürüş ve Şerit Takibi (DRIVING Phase)
        if self.aligned:
            self.nav_state = 'DRIVING'
            # Sapma toleransın 3 katını aşarsa yeniden hizalanmaya geç
            if abs(heading_error) > self.heading_tol * 3.0:
                self.aligned = False
                self.get_logger().info("⚠️ Açısal sapma arttı! Yeniden hizalanmaya geçiliyor.")
                v = 0.0
                w = 0.0
            else:
                # İleri hız (Hedefe yaklaşırken yumuşak yavaşlama)
                speed_ratio = min(1.0, distance / 3.0)  # Son 3 metrede yavaşlama
                v = max(0.15, self.max_linear_x * speed_ratio)
                
                # Şerit takip (Lane-keeping P control)
                w = self.kp_lane * heading_error
                w = max(-self.max_angular_z, min(self.max_angular_z, w))

        self.send_motor_cmd(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
