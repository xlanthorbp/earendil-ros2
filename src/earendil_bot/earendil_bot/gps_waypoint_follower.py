#!/usr/bin/env python3
"""
GPS Waypoint Follower Node
-----------------------------------------
Navigates the robot to a target GPS coordinate dynamically based on real-time
GPS data and IMU heading.

Usage:
  ros2 run earendil_bot gps_waypoint_follower --ros-args \
    -p target_lat:=39.925000 -p target_lon:=32.836000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
import math
import time
from earendil_bot.utils.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Hedef koordinatlar
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # Hız limitleri ve hassasiyetler
        self.declare_parameter('heading_tolerance', 0.122)  # ~7.0 derece
        self.declare_parameter('arrival_radius', 0.5)       # 0.5 metre (kullanıcı onayı)
        self.declare_parameter('max_linear_x', 0.6)         # İleri max hız
        self.declare_parameter('max_angular_z', 1.0)        # Dönüş max hızı

        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value

        self.get_logger().info(f"Target Waypoint: ({self.target_lat:.6f}, {self.target_lon:.6f})")
        self.get_logger().info(f"Arrival Radius: {self.arrival_radius}m")
        self.get_logger().info(f"Waiting for GPS on /gps/fix and IMU on /imu/data ...")

        # Durum Değişkenleri
        self.current_lat = None
        self.current_lon = None
        self.imu_heading = None
        
        self.last_imu_time = 0.0
        self.last_gps_time = 0.0
        
        self.aligned = False      # Araç hedefe döndü mü?
        self.arrived = False      # Araç hedefe vardı mı?

        # Publisher & Subscriber
        # Motor bridge cmd_vel dinliyor, twist_mux varsa cmd_vel_nav.
        # Biz doğrudan cmd_vel_nav yayınlayalım, eğer çalışmazsa cmd_vel eklenebilir.
        # Varsayılan olarak twist_mux kullanıldığı belirtilmişti
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        # Control loop 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_heading = math.atan2(siny_cosp, cosy_cosp)
        self.last_imu_time = time.time()

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = time.time()

    def control_loop(self):
        cmd = Twist()

        if self.arrived:
            return

        if self.target_lat == 0.0 and self.target_lon == 0.0:
            self.get_logger().warn("Hedef koordinat girilmedi! (0.0, 0.0). Bekleniyor...", throttle_duration_sec=3.0)
            return

        if self.imu_heading is None or (time.time() - self.last_imu_time > 1.5):
            self.get_logger().warn("IMU verisi bekleniyor veya koptu!", throttle_duration_sec=2.0)
            self.stop_robot(cmd)
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("GPS verisi bekleniyor veya koptu!", throttle_duration_sec=2.0)
            self.stop_robot(cmd)
            return

        # Anlık Hedef Açı ve Mesafe Hesabı
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        distance = haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)

        # Hedefe Ulaşma Kontrolü
        if distance <= self.arrival_radius:
            self.arrived = True
            self.get_logger().info(f"HEDEFE ULAŞILDI! Hedefe uzaklık: {distance:.2f}m")
            self.stop_robot(cmd)
            return

        # Heading Hatası (Radyan)
        error = angle_error_rad(target_bearing, self.imu_heading)

        self.get_logger().info(
            f"Mesafe: {distance:.1f}m | "
            f"Açı Hatası: {math.degrees(error):.1f}° | "
            f"Durum: {'SÜRÜŞ' if self.aligned else 'DÖNÜŞ'}", throttle_duration_sec=1.0)

        # AŞAMA 1: Hedefe Dönüş (Rotate phase)
        if not self.aligned:
            if abs(error) > self.heading_tol:
                # Sadece olduğu yerde dön (PID mantığı: error ile orantılı)
                kp_angular = 2.5
                angular_vel = kp_angular * error
                
                # Sınırlandırma
                if angular_vel > self.max_angular_z: angular_vel = self.max_angular_z
                elif angular_vel < -self.max_angular_z: angular_vel = -self.max_angular_z
                
                cmd.angular.z = angular_vel
                cmd.linear.x = 0.0
            else:
                self.aligned = True
                self.get_logger().info("Açı hizalandı! Sürüş aşamasına geçiliyor.")

        # AŞAMA 2: İleri Sürüş (Drive phase)
        if self.aligned:
            # Çok fazla sapma varsa tekrar sadece dönüş aşamasına dön (3 katı tolerans)
            if abs(error) > self.heading_tol * 3:
                self.aligned = False
                self.get_logger().info("Hizalanma bozuldu! Yeniden hizalanıyor...")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  # Bir sonraki döngüde hesaplanacak
            else:
                # İleri sürerken küçük düzeltmeler yap
                cmd.linear.x = self.max_linear_x
                cmd.angular.z = 1.5 * error  # Yolda tutma Kp'si
                
                # Z sınırlandırma (Gerekirse)
                if cmd.angular.z > self.max_angular_z: cmd.angular.z = self.max_angular_z
                elif cmd.angular.z < -self.max_angular_z: cmd.angular.z = -self.max_angular_z

        self.pub.publish(cmd)

    def stop_robot(self, cmd: Twist):
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
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
