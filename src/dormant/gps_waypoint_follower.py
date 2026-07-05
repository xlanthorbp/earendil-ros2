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
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Target coordinates
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # Speed limits and tolerances
        self.declare_parameter('heading_tolerance', 0.122)  # ~7.0 degrees
        self.declare_parameter('arrival_radius', 0.5)       # 0.5 meters (user confirmation)
        self.declare_parameter('max_linear_x', 0.6)         # Forward max speed
        self.declare_parameter('max_angular_z', 1.0)        # Turning max speed
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('dry_run', False)

        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.invert_turn = self.get_parameter('invert_turn').value
        self.dry_run = self.get_parameter('dry_run').value

        self.get_logger().info(f"Target Waypoint: ({self.target_lat:.6f}, {self.target_lon:.6f})")
        self.get_logger().info(f"Arrival Radius: {self.arrival_radius}m")
        self.get_logger().info(f"Waiting for GPS on /gps/fix and IMU on /imu/data ...")

        # State Variables
        self.current_lat = None
        self.current_lon = None
        self.imu_heading = None
        
        self.last_imu_time = 0.0
        self.last_gps_time = 0.0
        
        self.aligned = False      # Is the vehicle aligned to target?
        self.arrived = False      # Has the vehicle arrived at target?

        # Publisher & Subscriber
        # Motor bridge listens to cmd_vel, if twist_mux is present it's cmd_vel_nav.
        # We publish to cmd_vel_nav directly, if not working it can be changed to cmd_vel.
        # It was specified that twist_mux is used by default.
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
            self.get_logger().warn("Target coordinates not entered! (0.0, 0.0). Waiting...", throttle_duration_sec=3.0)
            return

        if self.imu_heading is None or (time.time() - self.last_imu_time > 1.5):
            self.get_logger().warn("Waiting for IMU data or connection lost!", throttle_duration_sec=2.0)
            self.stop_robot(cmd)
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("Waiting for GPS data or connection lost!", throttle_duration_sec=2.0)
            self.stop_robot(cmd)
            return

        # Current Target Bearing and Distance Calculation
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        distance = haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)

        # Target Arrival Check
        if distance <= self.arrival_radius:
            self.arrived = True
            self.get_logger().info(f"TARGET REACHED! Distance to target: {distance:.2f}m")
            self.stop_robot(cmd)
            return

        # Heading Error (Radians)
        error = angle_error_rad(target_bearing, self.imu_heading)

        self.get_logger().info(
            f"Distance: {distance:.1f}m | "
            f"Angle Error: {math.degrees(error):.1f}° | "
            f"State: {'DRIVING' if self.aligned else 'ROTATING'}", throttle_duration_sec=1.0)

        # PHASE 1: Rotate to Target (Rotate phase)
        if not self.aligned:
            if abs(error) > self.heading_tol:
                # Rotate in place (PID logic: proportional to error)
                kp_angular = 2.5
                angular_vel = kp_angular * error
                
                # Limit
                if angular_vel > self.max_angular_z: angular_vel = self.max_angular_z
                elif angular_vel < -self.max_angular_z: angular_vel = -self.max_angular_z
                
                cmd.angular.z = angular_vel
                cmd.linear.x = 0.0
            else:
                self.aligned = True
                self.get_logger().info("Angle aligned! Switching to driving phase.")

        # PHASE 2: Drive Forward (Drive phase)
        if self.aligned:
            # If deviation is too large, go back to rotation phase (3x tolerance)
            if abs(error) > self.heading_tol * 3:
                self.aligned = False
                self.get_logger().info("Alignment lost! Re-aligning...")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  # Will be calculated in the next loop
            else:
                # Make small corrections while driving forward
                cmd.linear.x = self.max_linear_x
                cmd.angular.z = 1.5 * error  # Lane keeping Kp
                
                # Z limit (If needed)
                if cmd.angular.z > self.max_angular_z: cmd.angular.z = self.max_angular_z
                elif cmd.angular.z < -self.max_angular_z: cmd.angular.z = -self.max_angular_z

        if self.invert_turn:
            cmd.angular.z = -cmd.angular.z

        if self.dry_run:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0

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
