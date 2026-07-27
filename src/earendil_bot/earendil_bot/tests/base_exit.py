#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float64
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class BaseExitNode(Node):
    def __init__(self):
        super().__init__('base_exit_node')
        
        # Parameters
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('clear_threshold', 1.5)  # Meters (Minimum distance to consider the 240-degree area clear)
        self.declare_parameter('arc_degrees', 120.0)    # Right and left angle relative to front center (120 + 120 = 240 degrees)
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.clear_threshold = self.get_parameter('clear_threshold').value
        self.arc_rad = math.radians(self.get_parameter('arc_degrees').value)
        
        # State variables
        self.state = 'DRIVING'
        self.current_yaw = 0.0
        self.base_saved = False
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(String, '/earendil/control/command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.heading_sub = self.create_subscription(Float64, '/earendil/heading/deg', self.heading_callback, 10)
        self.mag_heading_sub = self.create_subscription(Float32, '/mag/heading', self.mag_heading_callback, 10)
        
        # TF (To get X and Y position based on map/odom)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Base Exit Node Started. Driving forward...')
        
    def heading_callback(self, msg: Float64):
        # Heading from QMC5883L in degrees -> convert to radians [-pi, pi]
        deg = float(msg.data)
        self.current_yaw = math.radians(deg)
        # Normalize to [-pi, pi]
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

    def mag_heading_callback(self, msg: Float32):
        deg = float(msg.data)
        self.current_yaw = math.radians(deg)
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        
    def scan_callback(self, msg: LaserScan):
        if self.state == 'DONE':
            return
            
        if self.state == 'DRIVING':
            # Check the area between -120 and +120 degrees
            start_angle = -self.arc_rad
            end_angle = self.arc_rad
            
            is_clear = True
            min_dist = float('inf')
            
            for i, r in enumerate(msg.ranges):
                # Is the distance valid?
                if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                    continue
                    
                angle = msg.angle_min + i * msg.angle_increment
                
                # Is the angle between -120 and +120?
                if start_angle <= angle <= end_angle:
                    if r < min_dist:
                        min_dist = r
                    
                    if r < self.clear_threshold:
                        is_clear = False
                        # We do not break here so we can fully calculate min_dist
            
            if is_clear:
                self.get_logger().info(f'✅ FRONT AREA COMPLETELY CLEAR! (Min distance: {min_dist:.2f}m > {self.clear_threshold}m). Stopping motors...')
                
                # Stop
                msg = String()
                msg.data = "stop"
                self.cmd_pub.publish(msg)
                
                self.state = 'SAVING'
                self.save_position()
            else:
                # Continue driving forward
                self.get_logger().info(
                    f'Driving... [Nearest obstacle: {min_dist:.2f}m] | '
                    f'[Target Threshold: {self.clear_threshold}m] | '
                    f'[Current Compass Angle: {math.degrees(self.current_yaw)%360:.1f}°]',
                    throttle_duration_sec=1.0
                )
                msg = String()
                msg.data = "f 70"  # Target forward PWM
                self.cmd_pub.publish(msg)

    def save_position(self):
        # 1. Get position (Try Odom via TF first, otherwise assume 0.0)
        pos_x = 0.0
        pos_y = 0.0
        
        try:
            # Try map or odom first
            try:
                t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            except TransformException:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                
            pos_x = t.transform.translation.x
            pos_y = t.transform.translation.y
        except TransformException:
            self.get_logger().warn('TF (Odom/Map) not found! Saving X and Y as 0.0.')
            
        # 2. Reverse the rotation obtained from the Magnetometer (Add 180 degrees / Pi)
        reversed_yaw = self.current_yaw + math.pi
        
        # Convert new Yaw angle to Quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(reversed_yaw / 2.0)
        qw = math.cos(reversed_yaw / 2.0)
        
        # 3. Save to JSON file
        base_data = {
            'x': pos_x,
            'y': pos_y,
            'z': 0.0,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw,
        }
        
        save_path = os.path.expanduser('~/.ros/base_position.json')
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        with open(save_path, 'w') as f:
            json.dump(base_data, f, indent=2)
            
        self.get_logger().info(f'📍 Base Position Saved: x={pos_x:.2f}, y={pos_y:.2f}')
        self.get_logger().info(f'🔄 Original Yaw: {math.degrees(self.current_yaw):.1f}°, Saved (Reversed) Yaw: {math.degrees(reversed_yaw)%360:.1f}°')
        
        self.state = 'DONE'
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = BaseExitNode()
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
