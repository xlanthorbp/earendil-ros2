#!/usr/bin/env python3
"""
Magnetometer Turn Test — GPS Bearing
--------------------------------------
Enter robot and target GPS coordinates.
Calculates the bearing and rotates the rover toward the target
using magnetometer heading feedback.

Usage:
  ros2 run earendil_bot mag_turn_test --ros-args \
    -p robot_lat:=39.925000 -p robot_lon:=32.836000 \
    -p base_lat:=39.925500 -p base_lon:=32.837000
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math


class MagTurnTest(Node):
    def __init__(self):
        super().__init__('mag_turn_test')

        # GPS coordinates
        self.declare_parameter('robot_lat', 0.0)
        self.declare_parameter('robot_lon', 0.0)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)

        # Tolerances
        self.declare_parameter('heading_tolerance', 10.0)  # degrees
        self.declare_parameter('turn_speed', 0.5)          # rad/s

        robot_lat = self.get_parameter('robot_lat').value
        robot_lon = self.get_parameter('robot_lon').value
        base_lat = self.get_parameter('base_lat').value
        base_lon = self.get_parameter('base_lon').value
        self.tolerance = self.get_parameter('heading_tolerance').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Calculate target bearing from GPS coordinates
        self.target_bearing = self._calculate_bearing(
            robot_lat, robot_lon, base_lat, base_lon)
        distance = self._haversine(
            robot_lat, robot_lon, base_lat, base_lon)

        self.get_logger().info(f"Robot  : ({robot_lat:.6f}, {robot_lon:.6f})")
        self.get_logger().info(f"Hedef  : ({base_lat:.6f}, {base_lon:.6f})")
        self.get_logger().info(f"Mesafe : {distance:.1f} m")
        self.get_logger().info(f"Hedef Bearing : {self.target_bearing:.1f}°")
        self.get_logger().info(f"Tolerans      : {self.tolerance:.1f}°")
        self.get_logger().info("Manyetometre verisini bekliyorum (/mag/heading) ...")

        # State
        self.current_heading = None
        self.aligned = False

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(
            Float32, '/mag/heading', self.heading_cb, 10)

        # Control loop at 2 Hz
        self.timer = self.create_timer(0.5, self.control_loop)

    def heading_cb(self, msg: Float32):
        self.current_heading = msg.data

    def control_loop(self):
        cmd = Twist()

        if self.aligned:
            return

        if self.current_heading is None:
            self.get_logger().info(
                "Manyetometre verisi bekleniyor...",
                throttle_duration_sec=3.0)
            return

        # Calculate shortest angular error (in degrees)
        error = self.target_bearing - self.current_heading
        # Normalize to [-180, 180]
        error = (error + 180.0) % 360.0 - 180.0

        self.get_logger().info(
            f"Mevcut: {self.current_heading:.1f}° | "
            f"Hedef: {self.target_bearing:.1f}° | "
            f"Hata: {error:.1f}°")

        if abs(error) > self.tolerance:
            cmd.angular.z = self.turn_speed if error > 0 else -self.turn_speed
            cmd.linear.x = 0.0
        else:
            self.aligned = True
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            self.get_logger().info(
                f"HEDEFE ULASILDI! Mevcut: {self.current_heading:.1f}° ≈ "
                f"Hedef: {self.target_bearing:.1f}°")

        self.cmd_pub.publish(cmd)

    # ---- Math ----
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing in degrees (0=North, 90=East, 180=South, 270=West)."""
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        bearing_rad = math.atan2(x, y)
        return (math.degrees(bearing_rad) + 360.0) % 360.0

    def _haversine(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters between two GPS points."""
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def main(args=None):
    rclpy.init(args=args)
    node = MagTurnTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
