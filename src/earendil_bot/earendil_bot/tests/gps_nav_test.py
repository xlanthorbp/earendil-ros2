#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
"""
GPS Nav Test Node
-----------------------------------------
Navigates the robot to a target GPS coordinate dynamically based on real-time
GPS data (/gps/fix) and Magnetometer heading (/mag/heading).

Usage:
  ros2 run earendil_bot gps_nav_test --ros-args \
    -p target_lat:=39.925000 -p target_lon:=32.836000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import time
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class GpsNavTest(Node):
    def __init__(self):
        super().__init__('gps_nav_test')

        # Target coordinates
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # Speed limits and tolerances
        self.declare_parameter('heading_tolerance_deg', 7.0) # degrees
        self.declare_parameter('arrival_radius', 0.5)        # 0.5 meters (user confirmation)
        self.declare_parameter('max_linear_x', 0.6)          # Forward max speed
        self.declare_parameter('max_angular_z', 1.0)         # Turning max speed
        self.declare_parameter('kp_angular', 2.5)            # P-gain for rotation phase
        self.declare_parameter('kp_lane', 1.5)               # P-gain for lane keeping while driving
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('dry_run', False)

        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        heading_tol_deg = self.get_parameter('heading_tolerance_deg').value
        self.heading_tol = math.radians(heading_tol_deg)
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kp_lane = self.get_parameter('kp_lane').value
        self.invert_turn = self.get_parameter('invert_turn').value
        self.dry_run = self.get_parameter('dry_run').value

        self.get_logger().info(f"Target Waypoint: ({self.target_lat:.6f}, {self.target_lon:.6f})")
        self.get_logger().info(f"Arrival Radius: {self.arrival_radius}m")
        self.get_logger().info(f"Heading Tolerance: {heading_tol_deg:.1f}°")
        self.get_logger().info(f"Waiting for GPS on /gps/fix and Magnetometer on /mag/heading ...")

        # State Variables
        self.current_lat = None
        self.current_lon = None
        self.mag_heading = None
        
        self.last_mag_time = 0.0
        self.last_gps_time = 0.0
        
        self.aligned = False      # Is the vehicle aligned to target?
        self.arrived = False      # Has the vehicle arrived at target?

        # Publisher & Subscriber
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        # Control loop 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def mag_cb(self, msg: Float32):
        # /mag/heading publishes heading in degrees (0-360)
        self.mag_heading = math.radians(msg.data)
        self.last_mag_time = time.time()

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

        if self.mag_heading is None or (time.time() - self.last_mag_time > 1.5):
            self.get_logger().warn("Waiting for Magnetometer data (/mag/heading) or connection lost!", throttle_duration_sec=2.0)
            self.stop_robot(cmd)
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("Waiting for GPS data (/gps/fix) or connection lost!", throttle_duration_sec=2.0)
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
        error = angle_error_rad(target_bearing, self.mag_heading)

        self.get_logger().info(
            f"Distance: {distance:.1f}m | "
            f"Target Bearing: {math.degrees(target_bearing):.1f}° | "
            f"MAG: {math.degrees(self.mag_heading):.1f}° | "
            f"Error: {math.degrees(error):.1f}° | "
            f"State: {'DRIVING' if self.aligned else 'ROTATING'}", throttle_duration_sec=1.0)

        # PHASE 1: Rotate to Target (Rotate phase)
        if not self.aligned:
            if abs(error) > self.heading_tol:
                # Rotate in place (PID logic: proportional to error)
                angular_vel = self.kp_angular * error
                
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
                cmd.angular.z = self.kp_lane * error  # Lane keeping Kp
                
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
    node = GpsNavTest()
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

