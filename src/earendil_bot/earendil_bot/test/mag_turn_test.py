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
import time
from earendil_bot.utils.gps_math import bearing_between_gps_deg, haversine, angle_error_deg


class MagTurnTest(Node):
    def __init__(self):
        super().__init__('mag_turn_test')

        # GPS coordinates
        self.declare_parameter('robot_lat', 0.0)
        self.declare_parameter('robot_lon', 0.0)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)

        # Tolerances
        self.declare_parameter('heading_tolerance', 7.0)  # degrees
        self.declare_parameter('turn_speed', 1.5)         # rad/s (1.5 rad/s = 90 PWM)
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('dry_run', False)

        robot_lat = self.get_parameter('robot_lat').value
        robot_lon = self.get_parameter('robot_lon').value
        base_lat = self.get_parameter('base_lat').value
        base_lon = self.get_parameter('base_lon').value
        self.tolerance = self.get_parameter('heading_tolerance').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.invert_turn = self.get_parameter('invert_turn').value
        self.dry_run = self.get_parameter('dry_run').value

        # Calculate target bearing from GPS coordinates
        self.target_bearing = bearing_between_gps_deg(
            robot_lat, robot_lon, base_lat, base_lon)
        distance = haversine(
            robot_lat, robot_lon, base_lat, base_lon)

        self.get_logger().info(f"Robot  : ({robot_lat:.6f}, {robot_lon:.6f})")
        self.get_logger().info(f"Hedef  : ({base_lat:.6f}, {base_lon:.6f})")
        self.get_logger().info(f"Mesafe : {distance:.1f} m")
        self.get_logger().info(f"Hedef Bearing : {self.target_bearing:.1f}°")
        self.get_logger().info(f"Tolerans      : {self.tolerance:.1f}°")
        self.get_logger().info("Manyetometre verisini bekliyorum (/mag/heading) ...")

        # State
        self.current_heading = None
        self.last_mag_time = 0.0
        self.aligned = False

        # Publisher & Subscriber
        # twist_mux kullanıldığı için doğrudan cmd_vel yerine cmd_vel_nav'a yayın yapıyoruz
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(
            Float32, '/mag/heading', self.heading_cb, 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def heading_cb(self, msg: Float32):
        self.current_heading = msg.data
        self.last_mag_time = time.time()

    def control_loop(self):
        cmd = Twist()

        if self.aligned:
            return

        if self.current_heading is None or (time.time() - self.last_mag_time > 1.0):
            self.get_logger().warn(
                "Mag Watchdog triggered! Sensor lost or waiting.",
                throttle_duration_sec=3.0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Calculate shortest angular error (in degrees)
        error = angle_error_deg(self.target_bearing, self.current_heading)

        self.get_logger().info(
            f"Mevcut: {self.current_heading:.1f}° | "
            f"Hedef: {self.target_bearing:.1f}° | "
            f"Hata: {error:.1f}°")

        if abs(error) > self.tolerance:
            # P-Controller logic
            # Hata 25 dereceyken hızı 1.5 rad/s (yani 90 PWM) yapacak P katsayısı: 1.5 / 25 = 0.06
            kp = 0.06
            angular_vel = kp * error
            
            # Dönüş hızı çok yüksekse limitle (ROS Parametresinden gelir, varsayılan 0.5)
            if angular_vel > self.turn_speed: angular_vel = self.turn_speed
            elif angular_vel < -self.turn_speed: angular_vel = -self.turn_speed
            
            cmd.angular.z = angular_vel
            cmd.linear.x = 0.0
        else:
            self.aligned = True
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            self.get_logger().info(
                f"HEDEFE ULASILDI! Mevcut: {self.current_heading:.1f}° ≈ "
                f"Hedef: {self.target_bearing:.1f}°")

        if self.invert_turn:
            cmd.angular.z = -cmd.angular.z

        if self.dry_run:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    node = MagTurnTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
