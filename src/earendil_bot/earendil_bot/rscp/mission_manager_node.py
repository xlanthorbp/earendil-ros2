#!/usr/bin/env python3
"""
Ana Görev Yöneticisi ve Durum Makinesi (Master Mission Manager Node)
---------------------------------------------------------------------
RSCP köprü düğümünden gelen komutları alarak Otonom Nodelarımızı (peak_finder, gps_navigator_node, tunnel_test5, base_enter) eksiksiz koordine eder.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int32, Float64, String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Twist


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # Parametreler
        self.declare_parameter('default_search_radius', 5.0)
        self.declare_parameter('basalt_rock_arrival_distance', 0.5)
        self.declare_parameter('auto_stop_on_disarm', True)
        self.declare_parameter('task_finished_delay_s', 0.5)

        # Durum Değişkenleri
        self.current_stage = 0  # 1, 2, 3, 4
        self.is_armed = False

        self.search_center_lat = 0.0
        self.search_center_lon = 0.0
        self.search_radius = float(self.get_parameter('default_search_radius').value)
        self.basalt_rock_arrival_distance = float(self.get_parameter('basalt_rock_arrival_distance').value)
        self.auto_stop_on_disarm = bool(self.get_parameter('auto_stop_on_disarm').value)
        self.task_finished_delay_s = float(self.get_parameter('task_finished_delay_s').value)

        self.nav_target_lat = 0.0
        self.nav_target_lon = 0.0

        self.current_lat = None
        self.current_lon = None
        self.current_alt = 0.0

        # Subscriptions (RSCP Köprü Düğümünden Gelen Komutlar)
        self.create_subscription(Int32, '/rscp/command/set_stage', self.on_set_stage, 10)
        self.create_subscription(Bool, '/rscp/command/arm', self.on_arm_disarm, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_lat', self.on_search_lat, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_lon', self.on_search_lon, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_radius', self.on_search_radius, 10)
        self.create_subscription(Float64, '/rscp/command/navigate_gps_lat', self.on_nav_lat, 10)
        self.create_subscription(Float64, '/rscp/command/navigate_gps_lon', self.on_nav_lon, 10)
        self.create_subscription(Empty, '/rscp/command/start_exploration', self.on_start_exploration, 10)

        # Subscriptions (Navigasyon ve Sensör Geri Bildirimleri)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Bool, '/navigator/arrived', self.on_navigator_arrived, 10)
        self.create_subscription(NavSatFix, '/gps/peak_coordinate', self.on_peak_found, 10)
        self.create_subscription(Bool, '/rock_visible', self.on_rock_visible, 10)
        self.create_subscription(Point, '/rock_point', self.on_rock_point, 10)

        # Publishers (Navigasyon ve Hakem Geri Bildirimleri)
        self.pub_nav_goal = self.create_publisher(NavSatFix, '/navigator/goal', 10)
        self.pub_ack = self.create_publisher(Empty, '/rscp/feedback/ack', 10)
        self.pub_task_finished = self.create_publisher(Empty, '/rscp/feedback/task_finished', 10)
        self.pub_gps_coord = self.create_publisher(NavSatFix, '/rscp/feedback/gps_coordinate', 10)
        self.pub_distance = self.create_publisher(Float64, '/rscp/feedback/distance', 10)

        # Alt Görev Tetikleyicileri (Submodule Triggers)
        self.pub_tunnel_start = self.create_publisher(Empty, '/tunnel/start', 10)
        self.pub_base_enter_start = self.create_publisher(Empty, '/base_enter/start', 10)
        self.pub_peak_finder_start = self.create_publisher(NavSatFix, '/gps/search_center', 10)

        # Motor Komut Yayıncıları
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_h7_cmd = self.create_publisher(String, '/earendil/control/command', 10)

        # Algılama Değişkenleri
        self.rock_visible = False
        self.rock_distance = 99.0
        self.rock_found_sent = False

        self.get_logger().info("🎯 Mission Manager Node aktif. gps_navigator_node ile entegre çalışıyor.")

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def on_set_stage(self, msg: Int32):
        self.current_stage = msg.data
        self.get_logger().info(f"📍 Görev Aşaması Değiştirildi: STAGE {self.current_stage}")

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)
        stop_msg = String()
        stop_msg.data = "stop"
        self.pub_h7_cmd.publish(stop_msg)

    def on_arm_disarm(self, msg: Bool):
        self.is_armed = msg.data
        if self.is_armed:
            self.get_logger().info("🟢 Araç Otonom Çalışma İçin ARMED Edildi.")
        else:
            self.get_logger().warn("🛑 Araç DISARM / REMOTE STOP Edildi. Motorlar Durduruluyor.")
            self.stop_robot()

    def on_search_lat(self, msg: Float64):
        self.search_center_lat = msg.data

    def on_search_lon(self, msg: Float64):
        self.search_center_lon = msg.data

    def on_search_radius(self, msg: Float64):
        self.search_radius = msg.data
        self.get_logger().info(
            f"📍 Arama Alanı İsteyi Alındı: Merkez=({self.search_center_lat:.6f}, {self.search_center_lon:.6f}), Yarıçap={self.search_radius:.1f}m"
        )
        self.trigger_search_mission()

    def trigger_search_mission(self):
        if not self.is_armed:
            self.get_logger().warn("Arama isteği alındı fakat araç DISARM konumunda!")
            return

        # Navigasyon düğümüne hedef merkez noktasını gönder
        goal_msg = NavSatFix()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'gps'
        goal_msg.latitude = self.search_center_lat
        goal_msg.longitude = self.search_center_lon
        self.pub_nav_goal.publish(goal_msg)

        # STAGE 1: Zirve Arama Görevi
        if self.current_stage == 1:
            self.get_logger().info("🚀 Stage 1: gps_navigator_node ile Anten Alanı Merkezine Gidiliyor...")
            # peak_finder, merkeze varıldığında on_navigator_arrived tarafından başlatılacak

        # STAGE 2: Shackleton Krateri İlmenit Basalt Arama Görevi
        elif self.current_stage == 2:
            self.get_logger().info("🚀 Stage 2: gps_navigator_node ile Shackleton Krateri Merkezine Gidiliyor...")
            self.rock_found_sent = False

    def on_nav_lat(self, msg: Float64):
        self.nav_target_lat = msg.data

    def on_nav_lon(self, msg: Float64):
        self.nav_target_lon = msg.data
        self.get_logger().info(
            f"📍 NavigateToGPS İsteği Alındı: Hedef=({self.nav_target_lat:.6f}, {self.nav_target_lon:.6f})"
        )
        self.trigger_navigation_mission()

    def trigger_navigation_mission(self):
        if not self.is_armed:
            self.get_logger().warn("Navigasyon isteği alındı fakat araç DISARM konumunda!")
            return

        # gps_navigator_node için hedef yayınla
        goal_msg = NavSatFix()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'gps'
        goal_msg.latitude = self.nav_target_lat
        goal_msg.longitude = self.nav_target_lon
        self.pub_nav_goal.publish(goal_msg)

        # STAGE 3: Lava Tube Girişine Git
        if self.current_stage == 3:
            self.get_logger().info("🚀 Stage 3: gps_navigator_node ile Lava Tube Giriş Koordinatlarına Gidiliyor...")

        # STAGE 4: Airlock / Üs Binasına Dön
        elif self.current_stage == 4:
            self.get_logger().info("🚀 Stage 4: gps_navigator_node ile Airlock Yaklaşma Bölgesine Gidiliyor...")

    def on_start_exploration(self, msg: Empty):
        self.get_logger().info("🚀 StartExploration Alındı! Stage 3: Tünel Keşfi Düğümü (tunnel_test5) Başlatılıyor...")
        self.pub_tunnel_start.publish(Empty())

    def on_navigator_arrived(self, msg: Bool):
        """gps_navigator_node hedefe vardığında tetiklenir."""
        if not msg.data or not self.is_armed:
            return

        self.get_logger().info(f"✅ gps_navigator_node hedefe ulaştı. Stage: {self.current_stage}")

        if self.current_stage == 1:
            # Merkeze varıldı -> Zirve arama modülünü (peak_finder) başlat
            self.get_logger().info("🏁 Anten Alanı merkezine ulaşıldı. Zirve Arama (peak_finder) başlatılıyor...")
            goal_msg = NavSatFix()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'gps'
            goal_msg.latitude = self.search_center_lat
            goal_msg.longitude = self.search_center_lon
            self.pub_peak_finder_start.publish(goal_msg)

        elif self.current_stage == 3:
            # Lava tube girişine varıldı ➔ Hakeme TaskFinished gönder
            self.get_logger().info("🏁 Lava tube girişine ulaşıldı. Hakeme TaskFinished iletiliyor...")
            time.sleep(self.task_finished_delay_s)
            self.pub_task_finished.publish(Empty())

        elif self.current_stage == 4:
            # Airlock yaklaşma bölgesine varıldı ➔ ArUco tabanlı Airlock park etme düğümünü başlat
            self.get_logger().info("🏁 Airlock yaklaşma bölgesine ulaşıldı. ArUco Otonom Park (base_enter) başlatılıyor...")
            self.pub_base_enter_start.publish(Empty())

    # Algılama ve Özel Görev Etkinlik Yöneticileri

    def on_peak_found(self, msg: NavSatFix):
        """peak_finder düğümü en yüksek altimetreye ulaştığında çağrılır."""
        if self.current_stage == 1:
            self.get_logger().info(f"🏔️ Zirve Bulundu: ({msg.latitude:.8f}, {msg.longitude:.8f}). Hakeme Gönderiliyor...")
            self.pub_gps_coord.publish(msg)
            time.sleep(self.task_finished_delay_s)
            self.pub_task_finished.publish(Empty())

    def on_rock_visible(self, msg: Bool):
        self.rock_visible = msg.data

    def on_rock_point(self, msg: Point):
        self.rock_distance = msg.z  # Metre cinsinden mesafe

        # Stage 2 Basalt Kayası Yanına Varış Kontrolü
        if self.current_stage == 2 and self.is_armed and not self.rock_found_sent:
            if self.rock_visible and self.rock_distance <= self.basalt_rock_arrival_distance:
                self.rock_found_sent = True
                self.get_logger().info(
                    f"🪨 İlmenit Basalt Kayasına Ulaşıldı! Mesafe: {self.rock_distance:.2f}m. Motorlar durdurulup GPS bildiriliyor..."
                )

                # Motorları durdur
                stop_msg = String()
                stop_msg.data = "stop"
                self.pub_h7_cmd.publish(stop_msg)

                # Güncel RTK GPS koordinatını hakeme ilet
                if self.current_lat is not None and self.current_lon is not None:
                    rock_fix = NavSatFix()
                    rock_fix.header.stamp = self.get_clock().now().to_msg()
                    rock_fix.header.frame_id = 'gps'
                    rock_fix.latitude = self.current_lat
                    rock_fix.longitude = self.current_lon
                    rock_fix.altitude = float(self.current_alt)
                    self.pub_gps_coord.publish(rock_fix)

                time.sleep(self.task_finished_delay_s)
                self.pub_task_finished.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
