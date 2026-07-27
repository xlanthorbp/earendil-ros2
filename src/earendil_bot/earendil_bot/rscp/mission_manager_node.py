#!/usr/bin/env python3
# Ana Görev Yöneticisi ve Durum Makinesi (Master Mission Manager)
# RSCP köprü düğümünden gelen komutları alarak Otonom Test Nodelarımızı (peak_finder, gps_nav_test, tunnel_test5, base_enter)
# ve algılama nodelarımızı (rock_receiver, aruco_receiver) koordine eder.

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int32, Float64, String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('default_search_radius', 5.0)
        self.declare_parameter('basalt_rock_arrival_distance', 0.5)
        self.declare_parameter('auto_stop_on_disarm', True)
        self.declare_parameter('task_finished_delay_s', 0.5)

        # State Variables
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

        # Subscriptions from RSCP Bridge Node
        self.create_subscription(Int32, '/rscp/command/set_stage', self.on_set_stage, 10)
        self.create_subscription(Bool, '/rscp/command/arm', self.on_arm_disarm, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_lat', self.on_search_lat, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_lon', self.on_search_lon, 10)
        self.create_subscription(Float64, '/rscp/command/search_area_radius', self.on_search_radius, 10)
        self.create_subscription(Float64, '/rscp/command/navigate_gps_lat', self.on_nav_lat, 10)
        self.create_subscription(Float64, '/rscp/command/navigate_gps_lon', self.on_nav_lon, 10)
        self.create_subscription(Empty, '/rscp/command/start_exploration', self.on_start_exploration, 10)

        # Subscriptions for Sensor Inputs
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(NavSatFix, '/gps/peak_coordinate', self.on_peak_found, 10)
        self.create_subscription(Bool, '/rock_visible', self.on_rock_visible, 10)
        self.create_subscription(Point, '/rock_point', self.on_rock_point, 10)

        # Publishers to RSCP Bridge Feedback Topics
        self.pub_ack = self.create_publisher(Empty, '/rscp/feedback/ack', 10)
        self.pub_task_finished = self.create_publisher(Empty, '/rscp/feedback/task_finished', 10)
        self.pub_gps_coord = self.create_publisher(NavSatFix, '/rscp/feedback/gps_coordinate', 10)
        self.pub_distance = self.create_publisher(Float64, '/rscp/feedback/distance', 10)

        # H7 Motor Override Command Publisher
        self.pub_h7_cmd = self.create_publisher(String, '/earendil/control/command', 10)

        # Perception Variables
        self.rock_visible = False
        self.rock_distance = 99.0
        self.rock_found_sent = False

        self.get_logger().info("Mission Manager Node initialized and listening for RSCP directives.")

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def on_set_stage(self, msg: Int32):
        self.current_stage = msg.data
        self.get_logger().info(f"📍 Mission Stage set to: STAGE {self.current_stage}")

    def on_arm_disarm(self, msg: Bool):
        self.is_armed = msg.data
        if self.is_armed:
            self.get_logger().info("🟢 Vehicle ARMED for Autonomous Operation.")
        else:
            self.get_logger().warn("🛑 Vehicle DISARMED / REMOTE STOP. Halting motors immediately.")
            stop_msg = String()
            stop_msg.data = "stop"
            self.pub_h7_cmd.publish(stop_msg)

    def on_search_lat(self, msg: Float64):
        self.search_center_lat = msg.data

    def on_search_lon(self, msg: Float64):
        self.search_center_lon = msg.data

    def on_search_radius(self, msg: Float64):
        self.search_radius = msg.data
        self.get_logger().info(
            f"Received Search Area Request: Center=({self.search_center_lat:.6f}, {self.search_center_lon:.6f}), Radius={self.search_radius:.1f}m"
        )
        self.trigger_search_mission()

    def trigger_search_mission(self):
        if not self.is_armed:
            self.get_logger().warn("Search request received but vehicle is disarmed!")
            return

        # STAGE 1: Peak Finding Mission
        if self.current_stage == 1:
            self.get_logger().info("🚀 Triggering Stage 1: Peak Finder Mission (peak_finder)...")
            # Signal peak finder params via system/ROS parameters if running
            pass

        # STAGE 2: Shackleton Crater Basalt Rock Detection
        elif self.current_stage == 2:
            self.get_logger().info("🚀 Triggering Stage 2: Basalt Rock Search Mission (rock_receiver)...")
            self.rock_found_sent = False

    def on_nav_lat(self, msg: Float64):
        self.nav_target_lat = msg.data

    def on_nav_lon(self, msg: Float64):
        self.nav_target_lon = msg.data
        self.get_logger().info(
            f"Received NavigateToGPS Request: Target=({self.nav_target_lat:.6f}, {self.nav_target_lon:.6f})"
        )
        self.trigger_navigation_mission()

    def trigger_navigation_mission(self):
        if not self.is_armed:
            self.get_logger().warn("Navigate request received but vehicle is disarmed!")
            return

        # STAGE 3: Navigate to Lava Tube Entrance
        if self.current_stage == 3:
            self.get_logger().info("🚀 Stage 3: Navigating to Lava Tube Entrance Coordinates...")

        # STAGE 4: Navigate to AirLock / Base Station
        elif self.current_stage == 4:
            self.get_logger().info("🚀 Stage 4: Navigating to Base Station Area...")

    def on_start_exploration(self, msg: Empty):
        self.get_logger().info("🚀 Received StartExploration! Triggering Stage 3: Tunnel Exploration (tunnel_test5)...")

    # Feedback Event Handlers

    def on_peak_found(self, msg: NavSatFix):
        """Called when peak_finder node locates peak altitude."""
        if self.current_stage == 1:
            self.get_logger().info(f"🏔️ Peak Found: ({msg.latitude:.8f}, {msg.longitude:.8f}). Forwarding to RSCP...")
            self.pub_gps_coord.publish(msg)
            time.sleep(0.5)
            self.pub_task_finished.publish(Empty())

    def on_rock_visible(self, msg: Bool):
        self.rock_visible = msg.data

    def on_rock_point(self, msg: Point):
        self.rock_distance = msg.z  # Distance in meters

        # Stage 2 Basalt Rock Arrival Check
        if self.current_stage == 2 and self.is_armed and not self.rock_found_sent:
            if self.rock_visible and self.rock_distance <= self.basalt_rock_arrival_distance:
                self.rock_found_sent = True
                self.get_logger().info(f"🪨 Basalt Rock Reached! Distance: {self.rock_distance:.2f}m. Stopping and reporting GPS...")

                # Stop motors
                stop_msg = String()
                stop_msg.data = "stop"
                self.pub_h7_cmd.publish(stop_msg)

                # Report current RTK GPS coordinate
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
