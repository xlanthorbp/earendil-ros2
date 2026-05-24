#!/usr/bin/env python3
"""
Pure GPS Navigation — Rotate Then Drive
-----------------------------------------
Enter robot and base coordinates as parameters.
The robot uses IMU to rotate toward the base, then drives forward.
Stops when it reaches the arrival radius.

Usage:
  ros2 run earendil_bot pure_gps_nav --ros-args \
    -p robot_lat:=39.925000 -p robot_lon:=32.836000 \
    -p base_lat:=39.925500 -p base_lon:=32.837000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math


class PureGpsNav(Node):
    def __init__(self):
        super().__init__('pure_gps_nav')

        # Robot position (manual entry)
        self.declare_parameter('robot_lat', 0.0)
        self.declare_parameter('robot_lon', 0.0)

        # Base/target position (manual entry)
        self.declare_parameter('base_lat', 0.0)
        self.declare_parameter('base_lon', 0.0)

        # Tolerances
        self.declare_parameter('heading_tolerance', 0.15)  # radians (~8.5 degrees)
        self.declare_parameter('arrival_radius', 2.0)      # meters

        robot_lat = self.get_parameter('robot_lat').value
        robot_lon = self.get_parameter('robot_lon').value
        base_lat = self.get_parameter('base_lat').value
        base_lon = self.get_parameter('base_lon').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.arrival_radius = self.get_parameter('arrival_radius').value

        # Calculate target bearing and distance once (coordinates are fixed)
        self.target_bearing = self._calculate_bearing(robot_lat, robot_lon, base_lat, base_lon)
        self.distance = self._haversine(robot_lat, robot_lon, base_lat, base_lon)

        self.get_logger().info(f"Robot : ({robot_lat:.6f}, {robot_lon:.6f})")
        self.get_logger().info(f"Base  : ({base_lat:.6f}, {base_lon:.6f})")
        self.get_logger().info(f"Distance : {self.distance:.1f} m")
        self.get_logger().info(f"Target Bearing : {math.degrees(self.target_bearing):.1f}°")
        self.get_logger().info(f"Waiting for IMU data on /imu/data ...")

        # State
        self.imu_heading = None
        self.aligned = False      # True when robot faces the target
        self.arrived = False      # True when robot reached the target

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

        if self.arrived:
            return

        if self.imu_heading is None:
            self.get_logger().info("Waiting for IMU...", throttle_duration_sec=3.0)
            return

        # Heading error
        error = self.target_bearing - self.imu_heading
        error = (error + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().info(
            f"IMU: {math.degrees(self.imu_heading):.1f}° | "
            f"Target: {math.degrees(self.target_bearing):.1f}° | "
            f"Error: {math.degrees(error):.1f}° | "
            f"Phase: {'DRIVE' if self.aligned else 'ROTATE'}")

        # PHASE 1: Rotate toward target
        if not self.aligned:
            if abs(error) > self.heading_tol:
                cmd.angular.z = 0.5 if error > 0 else -0.5
                cmd.linear.x = 0.0
            else:
                self.aligned = True
                self.get_logger().info("ALIGNED! Switching to DRIVE phase.")

        # PHASE 2: Drive forward with corrections
        if self.aligned:
            if abs(error) > self.heading_tol * 3:
                # Large deviation — stop and re-align
                self.aligned = False
                self.get_logger().info("Lost alignment! Re-rotating...")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 if error > 0 else -0.5
            else:
                # Drive forward with proportional steering correction
                cmd.linear.x = 0.5
                cmd.angular.z = 0.3 * error

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
    node = PureGpsNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
