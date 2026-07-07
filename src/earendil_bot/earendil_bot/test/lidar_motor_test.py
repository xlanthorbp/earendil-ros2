#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Bool
import math

class LidarMotorTestNode(Node):
    def __init__(self):
        super().__init__('lidar_motor_test')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ir_sub = self.create_subscription(Float32, '/infrared_distance', self.ir_callback, 10)
        
        # self.aruco_midpoint_sub = self.create_subscription(Point, '/aruco_midpoint', self.aruco_midpoint_callback, 10)
        # self.aruco_visible_sub = self.create_subscription(Bool, '/aruco_visible', self.aruco_visible_callback, 10)
        
        # State Variables
        self.previous_error = 0.0
        self.state = 'SEARCHING_ENTRANCE'
        self.max_ir_distance = -1.0
        self.tunnel_start_time = None
        
        # Aruco Variables
        # self.aruco_visible = False
        # self.aruco_angle = 0.0
        # self.aruco_distance = 0.0
        
        # Parameters
        self.declare_parameter('forward_speed_approach', 0.2)
        self.declare_parameter('forward_speed_tunnel', 0.3)
        self.declare_parameter('safety_speed', 0.1)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('safety_threshold', 0.25)
        self.declare_parameter('tunnel_detection_threshold', 0.3)
        self.declare_parameter('window_deg', 10.0)
        self.declare_parameter('kp_aruco', 0.02)
        
        self.forward_speed_approach = self.get_parameter('forward_speed_approach').value
        self.forward_speed_tunnel = self.get_parameter('forward_speed_tunnel').value
        self.safety_speed = self.get_parameter('safety_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.tunnel_detection_threshold = self.get_parameter('tunnel_detection_threshold').value
        self.window_deg = self.get_parameter('window_deg').value
        self.kp_aruco = self.get_parameter('kp_aruco').value
        
        # ROS 2 Shutdown Hook: Triggered when the node shuts down (error or normal)
        rclpy.get_default_context().on_shutdown(self.stop_motors_safely)
        
        self.get_logger().info('Lidar & Motor Test Node Started.')
        self.get_logger().info('State: SEARCHING_ENTRANCE (Turning right, looking for ArUco)')

    # def aruco_visible_callback(self, msg):
    #     self.aruco_visible = msg.data

    # def aruco_midpoint_callback(self, msg):
    #     self.aruco_angle = msg.x
    #     self.aruco_distance = msg.z

    def stop_motors_safely(self):
        """Sends 0 velocity to motors when the node shuts down for any reason."""
        self.get_logger().info('Shutdown signal received: Stopping motors for safety...')
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)

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

    def ir_callback(self, msg):
        dist = msg.data
        
        # If the incoming data is less than 20 or greater than 150, ignore it (set to -1)
        if dist < 20.0 or dist > 150.0:
            dist = -1.0
            
        if self.state == 'IN_TUNNEL' and self.tunnel_start_time is not None:
            elapsed_sec = (self.get_clock().now() - self.tunnel_start_time).nanoseconds / 1e9
            if elapsed_sec > 1.0:
                if dist != -1.0 and dist > self.max_ir_distance:
                    self.max_ir_distance = dist

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
            # --- CAMERA (ARUCO) SECTION COMMENTED OUT ---
            # if self.aruco_visible:
            #     self.get_logger().info('Entrance ArUco detected! Switching to ALIGNING_IN_PLACE.')
            #     self.state = 'ALIGNING_IN_PLACE'
            # else:
            #     self.get_logger().info('Searching for entrance ArUco...', throttle_duration_sec=2.0)
            #     twist.angular.z = -0.5 # Turn right slowly
                
        # elif self.state == 'ALIGNING_IN_PLACE':
        #     if self.aruco_visible:
        #         if abs(self.aruco_angle) > 3.0:  # If deviation is more than 3 degrees
        #             self.get_logger().info(f'Aligning in place... Angle: {self.aruco_angle:.1f}deg', throttle_duration_sec=1.0)
        #             twist.linear.x = 0.0
        #             
        #             angular_speed = -self.kp_aruco * self.aruco_angle
        #             # Minimum limit so rotation speed does not slow down too much (0.15 rad/s)
        #             if angular_speed > 0 and angular_speed < 0.15:
        #                 angular_speed = 0.15
        #             elif angular_speed < 0 and angular_speed > -0.15:
        #                 angular_speed = -0.15
        #                 
        #             twist.angular.z = angular_speed
        #         else:
        #             self.get_logger().info('Perfectly aligned to the center! Just going straight (APPROACHING_ENTRANCE).')
        #             self.state = 'APPROACHING_ENTRANCE'
        #     else:
        #         self.get_logger().warn('ArUco lost while aligning! Searching again.', throttle_duration_sec=1.0)
        #         self.state = 'SEARCHING_ENTRANCE'
                
        # elif self.state == 'APPROACHING_ENTRANCE':
            # Old code: Just go straight and check walls
            if left_dist < self.tunnel_detection_threshold and right_dist < self.tunnel_detection_threshold:
                self.get_logger().info(f'---> WALLS DETECTED! (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). ENTERED TUNNEL!')
                self.state = 'IN_TUNNEL'
                self.tunnel_start_time = self.get_clock().now()
                self.max_ir_distance = -1.0  # Reset maximum when entering the tunnel
            else:
                twist.linear.x = self.forward_speed_approach
                twist.angular.z = 0.0
                
        elif self.state == 'IN_TUNNEL':
            # Check if we exited the tunnel
            if left_dist >= self.tunnel_detection_threshold and right_dist >= self.tunnel_detection_threshold:
                self.get_logger().info(f'---> WALLS ENDED! (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). EXITED TUNNEL!')
                
                if self.max_ir_distance != -1.0:
                    self.get_logger().info(f'\n======================================================\n'
                                           f'TASK COMPLETED! MAXIMUM TUNNEL HEIGHT DETECTED: {self.max_ir_distance:.2f} m\n'
                                           f'======================================================\n')
                else:
                    self.get_logger().warn("No valid height measurement received or it lasted less than 1 second!")
                
                self.state = 'COMPLETED'
            else:
                # Simple escape logic (no PD)
                if left_dist < self.safety_threshold:
                    self.get_logger().warn('Too close to the left wall! Turning right.', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.15  # Turn right slowly
                elif right_dist < self.safety_threshold:
                    self.get_logger().warn('Too close to the right wall! Turning left.', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.15   # Turn left slowly
                else:
                    # When distance is over 25 cm, stop turning, go straight
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = 0.0
                    
        elif self.state == 'COMPLETED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        tunnel_height_m = self.max_ir_distance if self.max_ir_distance != -1.0 else 0.0
        
        # Regular status log (prints 2 times a second)
        self.get_logger().info(
            f'[{self.state}] Dist-> L: {left_dist:.2f}m, R: {right_dist:.2f}m, F: {front_dist:.2f}m | '
            f'Tunnel Height: {tunnel_height_m:.2f}m | '
            f'Motor-> Fwd: {twist.linear.x:.2f}, Turn: {twist.angular.z:.2f}',
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
