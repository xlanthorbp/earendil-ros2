#!/usr/bin/env python3
"""
IMU Heading Test — Rotate Toward Base
---------------------------------------
Manual coordinate entry version. No GPS hardware needed.
Enter robot and base coordinates as parameters,
the robot uses IMU to rotate toward the base.

Usage:
  ros2 run earendil_bot imu_heading_test --ros-args \
    -p robot_lat:=39.925000 -p robot_lon:=32.836000 \
    -p base_lat:=39.925500 -p base_lon:=32.837000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math


class ImuHeadingTest(Node):
    def __init__(self):
        super().__init__('imu_heading_test')

        # Robot position (manual entry)
        self.declare_parameter('robot_lat', 0.0)
        self.declare_parameter('robot_lon', 0.0)

        # Base/target position (manual entry)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)

        # Tolerance
        self.declare_parameter('heading_tolerance', 0.15)  # ~8.5 degrees

        robot_lat = self.get_parameter('robot_lat').value
        robot_lon = self.get_parameter('robot_lon').value
        base_lat = self.get_parameter('base_lat').value
        base_lon = self.get_parameter('base_lon').value
        self.heading_tol = self.get_parameter('heading_tolerance').value

        # Calculate target bearing once (coordinates are fixed)
        self.target_bearing = self._calculate_bearing(robot_lat, robot_lon, base_lat, base_lon)
        dist = self._haversine(robot_lat, robot_lon, base_lat, base_lon)

        self.get_logger().info(f"Robot : ({robot_lat:.6f}, {robot_lon:.6f})")
        self.get_logger().info(f"Base  : ({base_lat:.6f}, {base_lon:.6f})")
        self.get_logger().info(f"Distance : {dist:.1f} m")
        self.get_logger().info(f"Target Bearing : {math.degrees(self.target_bearing):.1f}°")
        self.get_logger().info(f"Waiting for IMU data on /imu/data ...")

        # State
        self.imu_heading = None

        # Publisher & Subscriber
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Control loop 2 Hz
        self.timer = self.create_timer(0.5, self.control_loop)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_heading = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        cmd = Twist()

        if self.imu_heading is None:
            self.get_logger().info("Waiting for IMU...", throttle_duration_sec=3.0)
            return

        # Heading error
        error = self.target_bearing - self.imu_heading
        error = (error + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().info(
            f"IMU: {math.degrees(self.imu_heading):.1f}° | "
            f"Target: {math.degrees(self.target_bearing):.1f}° | "
            f"Error: {math.degrees(error):.1f}°")

        if abs(error) > self.heading_tol:
            cmd.angular.z = 0.5 if error > 0 else -0.5
        else:
            self.get_logger().info("ALIGNED WITH BASE!")
            cmd.angular.z = 0.0

        cmd.linear.x = 0.0
        self.pub.publish(cmd)

    # ---- Math ----
    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
        return math.atan2(x, y)


def main(args=None):
    rclpy.init(args=args)
    node = ImuHeadingTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
