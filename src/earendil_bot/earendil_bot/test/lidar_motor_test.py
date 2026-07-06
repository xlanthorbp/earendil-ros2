#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class LidarMotorTestNode(Node):
    def __init__(self):
        super().__init__('lidar_motor_test')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # State Variables
        self.previous_error = 0.0
        self.state = 'SEARCHING_ENTRANCE'
        
        # Parameters
        self.declare_parameter('forward_speed_approach', 0.2)
        self.declare_parameter('forward_speed_tunnel', 0.3)
        self.declare_parameter('safety_speed', 0.1)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('safety_threshold', 0.1)
        self.declare_parameter('tunnel_detection_threshold', 0.3)
        self.declare_parameter('window_deg', 10.0)
        
        self.forward_speed_approach = self.get_parameter('forward_speed_approach').value
        self.forward_speed_tunnel = self.get_parameter('forward_speed_tunnel').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.tunnel_detection_threshold = self.get_parameter('tunnel_detection_threshold').value
        self.window_deg = self.get_parameter('window_deg').value
        
        self.get_logger().info('Lidar & Motor Test Node Started.')
        self.get_logger().info('State: SEARCHING_ENTRANCE (Moving forward, looking for walls)')

    def get_avg_distance(self, msg, target_angle_deg):
        target_rad = math.radians(target_angle_deg)
        window_rad = math.radians(self.window_deg)
        
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            diff = abs(math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad)))
            if diff <= window_rad / 2.0:
                valid_ranges.append(r)
                
        if len(valid_ranges) == 0:
            return 6.0
        return sum(valid_ranges) / len(valid_ranges)

    def scan_callback(self, msg):
        front_dist = self.get_avg_distance(msg, 0.0)
        twist = Twist()
        
        if front_dist < 0.20 and self.state != 'COMPLETED':
            self.get_logger().warn(f'OBSTACLE AHEAD ({front_dist:.2f}m)! Stopping.', throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
            
        left_dist = self.get_avg_distance(msg, 90.0)
        right_dist = self.get_avg_distance(msg, -90.0)
        
        if self.state == 'SEARCHING_ENTRANCE':
            # Check if walls are detected on both sides
            if left_dist < self.tunnel_detection_threshold and right_dist < self.tunnel_detection_threshold:
                self.get_logger().info(f'---> DUVARLAR ALGILANDI! (Sol: {left_dist:.2f}m, Sag: {right_dist:.2f}m). TUNELE GIRILDI!')
                self.state = 'IN_TUNNEL'
            else:
                twist.linear.x = self.forward_speed_approach
                twist.angular.z = 0.0
                
        elif self.state == 'IN_TUNNEL':
            # Check if we exited the tunnel
            if left_dist >= self.tunnel_detection_threshold and right_dist >= self.tunnel_detection_threshold:
                self.get_logger().info(f'---> DUVARLAR BITTI! (Sol: {left_dist:.2f}m, Sag: {right_dist:.2f}m). TUNELDEN CIKILDI!')
                self.state = 'COMPLETED'
            else:
                # Centering logic
                if left_dist < self.safety_threshold:
                    self.get_logger().warn('Sol duvara cok yakin!', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.8
                elif right_dist < self.safety_threshold:
                    self.get_logger().warn('Sag duvara cok yakin!', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.8
                else:
                    error = left_dist - right_dist
                    derivative = error - self.previous_error
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = (self.kp * error) + (self.kd * derivative)
                    self.previous_error = error
                    
        elif self.state == 'COMPLETED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        # Duzenli durum logu (saniyede 2 kez yazar)
        self.get_logger().info(
            f'[{self.state}] Mesafe-> Sol: {left_dist:.2f}m, Sag: {right_dist:.2f}m, On: {front_dist:.2f}m | '
            f'Motor-> Ileri: {twist.linear.x:.2f}, Donus: {twist.angular.z:.2f}',
            throttle_duration_sec=0.5
        )
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
