#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# Base Exit Node tarafından kaydedilen baz istasyonu konumunu kullanarak robotu güvenli bir şekilde şarj istasyonuna döndürmek için tasarlanmıştır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float64, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class BaseEnterNode(Node):
    def __init__(self):
        super().__init__('base_enter_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('arrival_radius', 0.15)       # meters
        self.declare_parameter('heading_tolerance', 0.08)     # ~4.5 degrees
        self.declare_parameter('lidar_threshold', 1.0)        # meters
        self.declare_parameter('lidar_stop_above', True)       # True: stop when > threshold, False: stop when < threshold
        self.declare_parameter('lidar_stop_ratio', 0.8)       # Ratio of ranges meeting the condition
        self.declare_parameter('min_enter_duration', 2.0)     # Avoid immediate stop when starting
        self.declare_parameter('wait_for_trigger', True)       # Wait for /base_enter/start trigger

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.lidar_threshold = self.get_parameter('lidar_threshold').value
        self.lidar_stop_above = self.get_parameter('lidar_stop_above').value
        self.lidar_stop_ratio = self.get_parameter('lidar_stop_ratio').value
        self.min_enter_duration = self.get_parameter('min_enter_duration').value
        self.wait_for_trigger = self.get_parameter('wait_for_trigger').value

        # ---------------------------------------------------------
        # State Variables
        # ---------------------------------------------------------
        # WAITING_FOR_TRIGGER -> NAVIGATING -> ALIGNING -> ENTERING -> DONE
        self.state = 'WAITING_FOR_TRIGGER' if self.wait_for_trigger else 'NAVIGATING'
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.entering_start_time = None

        # Load Saved Base Position
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self._load_base_position()

        # ---------------------------------------------------------
        # Publishers and Subscribers
        # ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(String, '/earendil/control/command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.heading_sub = self.create_subscription(Float32, '/mag/heading', self.heading_callback, 10)
        self.start_sub = self.create_subscription(Empty, '/base_enter/start', self.start_cb, 10)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Shutdown safety hook
        rclpy.get_default_context().on_shutdown(self.stop_robot_completely)

        self.get_logger().info(f'Base Enter Node Started. Initial State: {self.state}')

    def start_cb(self, msg: Empty):
        self.get_logger().info("🚀 Base enter trigger received from mission_manager_node!")
        if self.state == 'WAITING_FOR_TRIGGER':
            self.state = 'NAVIGATING'

    def _load_base_position(self):
        save_path = os.path.expanduser('~/.ros/base_position.json')
        if not os.path.exists(save_path):
            self.get_logger().error(f"❌ Base position file not found at {save_path}! Please run base_exit node first to save the base station location.")
            raise SystemExit
        
        try:
            with open(save_path, 'r') as f:
                base_data = json.load(f)
            
            self.target_x = base_data.get('x', 0.0)
            self.target_y = base_data.get('y', 0.0)
            
            # Convert Quaternion back to Yaw
            qx = base_data.get('qx', 0.0)
            qy = base_data.get('qy', 0.0)
            qz = base_data.get('qz', 0.0)
            qw = base_data.get('qw', 1.0)
            
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            self.target_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.get_logger().info(f"📍 Loaded Target Base Position: x={self.target_x:.2f}, y={self.target_y:.2f}, target_yaw={math.degrees(self.target_yaw):.1f}°")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load base position from file: {e}")
            raise SystemExit

    def heading_callback(self, msg: Float32):
        # Heading from QMC5883L in degrees -> convert to radians [-pi, pi]
        deg = float(msg.data)
        self.current_yaw = math.radians(deg)
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

    def angle_error_rad(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def send_motor_cmd(self, v, w):
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        msg = String()
        if v == 0.0 and w == 0.0:
            msg.data = "stop"
        elif v > 0:
            msg.data = "f 70"
        elif v < 0:
            msg.data = "b 70"
        elif w > 0:
            msg.data = "l 70"
        elif w < 0:
            msg.data = "r 70"
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.state in ('DONE', 'WAITING_FOR_TRIGGER'):
            return

        # Try to lookup current position from TF
        try:
            try:
                t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            except TransformException:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}. Odometry disabled, skipping NAVIGATING...', throttle_duration_sec=3.0)
            if self.state == 'NAVIGATING':
                self.state = 'ALIGNING'

        if self.state == 'NAVIGATING':
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_bearing = math.atan2(dy, dx)

            # Check arrival
            if distance <= self.arrival_radius:
                self.get_logger().info(f"📍 Arrived at target coordinate! Distance: {distance:.2f}m. Aligning to entry yaw...")
                self.state = 'ALIGNING'
                self.send_motor_cmd(0.0, 0.0)  # Stop
                return

            error = self.angle_error_rad(target_bearing, self.current_yaw)
            self.get_logger().info(
                f"Navigating -> Dist: {distance:.2f}m | Yaw Error: {math.degrees(error):.1f}°",
                throttle_duration_sec=1.0
            )

            # Control strategy: Rotate in place if error is large, else move forward
            if abs(error) > 0.3:  # ~17 degrees
                self.send_motor_cmd(0.0, self.turn_speed if error > 0 else -self.turn_speed)
            else:
                self.send_motor_cmd(self.forward_speed, 0.0)

        elif self.state == 'ALIGNING':
            error = self.angle_error_rad(self.target_yaw, self.current_yaw)
            self.get_logger().info(
                f"Aligning -> Target Yaw: {math.degrees(self.target_yaw):.1f}° | Error: {math.degrees(error):.1f}°",
                throttle_duration_sec=1.0
            )

            if abs(error) > self.heading_tolerance:
                self.send_motor_cmd(0.0, self.turn_speed if error > 0 else -self.turn_speed)
            else:
                self.get_logger().info("✅ Alignment complete! Starting entry into the base station.")
                self.state = 'ENTERING'
                self.entering_start_time = self.get_clock().now()
                self.send_motor_cmd(0.0, 0.0)  # Stop briefly before entry

        elif self.state == 'ENTERING':
            self.get_logger().info(
                f"Entering base... Driving straight at {self.forward_speed:.2f} m/s",
                throttle_duration_sec=1.0
            )
            self.send_motor_cmd(self.forward_speed, 0.0)

    def scan_callback(self, msg: LaserScan):
        if self.state != 'ENTERING' or self.entering_start_time is None:
            return

        # Enforce minimum enter duration before parsing stop condition
        elapsed = (self.get_clock().now() - self.entering_start_time).nanoseconds / 1e9
        if elapsed < self.min_enter_duration:
            self.get_logger().info(
                f"In entrance zone... Ignoring Lidar check for another {self.min_enter_duration - elapsed:.1f}s",
                throttle_duration_sec=1.0
            )
            return

        valid_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            valid_ranges.append(r)

        if not valid_ranges:
            self.get_logger().warn("Lidar scanning returned no valid ranges!", throttle_duration_sec=2.0)
            return

        # Check threshold condition
        above_count = sum(1 for r in valid_ranges if r > self.lidar_threshold)
        ratio = above_count / len(valid_ranges)

        if self.lidar_stop_above:
            condition_met = (ratio >= self.lidar_stop_ratio)
            cond_str = "above"
        else:
            condition_met = ((1.0 - ratio) >= self.lidar_stop_ratio)
            cond_str = "under"

        self.get_logger().info(
            f"Lidar Status: {above_count}/{len(valid_ranges)} ({ratio*100:.1f}%) values > {self.lidar_threshold}m",
            throttle_duration_sec=0.5
        )

        if condition_met:
            self.get_logger().info(
                f"🛑 STOP TRIGGERED! {ratio*100:.1f}% of Lidar ranges are {cond_str} {self.lidar_threshold}m. Base entrance complete."
            )
            self.state = 'DONE'
            self.stop_robot_completely()
            raise SystemExit

    def stop_robot_completely(self):
        self.send_motor_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = BaseEnterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
