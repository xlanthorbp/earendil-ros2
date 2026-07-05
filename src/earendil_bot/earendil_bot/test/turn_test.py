#!/usr/bin/env python3
"""
Turn Test — Rotate Toward Base
---------------------------------------
Manual coordinate entry version. No GPS hardware needed.
Enter robot and base coordinates as parameters,
the robot uses IMU to rotate toward the base.

Usage:
  ros2 run earendil_bot turn_test --ros-args \
    -p robot_lat:=39.925000 -p robot_lon:=32.836000 \
    -p base_lat:=39.925500 -p base_lon:=32.837000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class TurnTest(Node):
    def __init__(self):
        super().__init__('turn_test')

        # Robot position (manual entry)
        self.declare_parameter('robot_lat', 0.0)
        self.declare_parameter('robot_lon', 0.0)

        # Base/target position (manual entry)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)

        self.declare_parameter('heading_tolerance', 0.15)  # ~8.5 degrees
        self.declare_parameter('turn_speed', 0.5)          # rad/s
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('dry_run', False)

        robot_lat = self.get_parameter('robot_lat').value
        robot_lon = self.get_parameter('robot_lon').value
        base_lat = self.get_parameter('base_lat').value
        base_lon = self.get_parameter('base_lon').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.invert_turn = self.get_parameter('invert_turn').value
        self.dry_run = self.get_parameter('dry_run').value

        # Calculate target bearing once (coordinates are fixed)
        self.target_bearing = bearing_between_gps_rad(robot_lat, robot_lon, base_lat, base_lon)
        dist = haversine(robot_lat, robot_lon, base_lat, base_lon)

        self.get_logger().info(f"Robot : ({robot_lat:.6f}, {robot_lon:.6f})")
        self.get_logger().info(f"Base  : ({base_lat:.6f}, {base_lon:.6f})")
        self.get_logger().info(f"Distance : {dist:.1f} m")
        self.get_logger().info(f"Target Bearing : {math.degrees(self.target_bearing):.1f}°")
        self.get_logger().info(f"Waiting for IMU data on /imu/data ...")

        # State
        self.imu_heading = None
        self.last_imu_time = 0.0

        # Publisher & Subscriber
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Control loop 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_heading = math.atan2(siny_cosp, cosy_cosp)
        self.last_imu_time = time.time()

    def control_loop(self):
        cmd = Twist()

        if self.imu_heading is None or (time.time() - self.last_imu_time > 1.0):
            self.get_logger().warn("IMU Watchdog triggered! Sensor lost or waiting.", throttle_duration_sec=3.0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # Heading error
        error = angle_error_rad(self.target_bearing, self.imu_heading)

        self.get_logger().info(
            f"IMU: {math.degrees(self.imu_heading):.1f}° | "
            f"Target: {math.degrees(self.target_bearing):.1f}° | "
            f"Error: {math.degrees(error):.1f}°")

        if abs(error) > self.heading_tol:
            # P-Controller logic
            kp = 2.0
            angular_vel = kp * error
            if angular_vel > self.turn_speed: angular_vel = self.turn_speed
            elif angular_vel < -self.turn_speed: angular_vel = -self.turn_speed
            cmd.angular.z = angular_vel
        else:
            self.get_logger().info("ALIGNED WITH BASE!")
            cmd.angular.z = 0.0

        if self.invert_turn:
            cmd.angular.z = -cmd.angular.z

        if self.dry_run:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            
        cmd.linear.x = 0.0
        self.pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    node = TurnTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
