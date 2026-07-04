import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class TunnelNavigatorNode(Node):
    def __init__(self):
        super().__init__('tunnel_navigator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.height_pub = self.create_publisher(Float32, '/current_tunnel_height', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.top_sonar_sub = self.create_subscription(Range, '/ir_top', self.top_sonar_callback, 10)
        
        # Tunnel state and measurement variables
        self.max_tunnel_height = 0.0
        self.tunnel_start_time = None
        self.state = 'APPROACHING'
        self.previous_error = 0.0  # Memory for the previous error used in the PD controller
        
        # Declare parameters
        self.declare_parameter('forward_speed_approach', 0.2)
        self.declare_parameter('forward_speed_tunnel', 0.3)
        self.declare_parameter('safety_speed', 0.1)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('safety_threshold', 0.1)
        self.declare_parameter('tunnel_detection_threshold', 0.3)
        self.declare_parameter('sensor_height_from_ground', 1.0)
        self.declare_parameter('window_deg', 10.0)
        
        # Read parameters
        self.forward_speed_approach = self.get_parameter('forward_speed_approach').value
        self.forward_speed_tunnel = self.get_parameter('forward_speed_tunnel').value
        self.safety_speed = self.get_parameter('safety_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.tunnel_detection_threshold = self.get_parameter('tunnel_detection_threshold').value
        self.sensor_height_from_ground = self.get_parameter('sensor_height_from_ground').value
        self.window_deg = self.get_parameter('window_deg').value
        
        self.get_logger().info('Tunnel Navigation Node Started.')
        self.get_logger().info('State: APPROACHING (Approaching the tunnel...)')

    def top_sonar_callback(self, msg):
        # Only process data when inside the tunnel
        if self.state == 'IN_TUNNEL' and self.tunnel_start_time is not None:
            # Calculate elapsed time in seconds since entering the tunnel
            elapsed_time = (self.get_clock().now() - self.tunnel_start_time).nanoseconds / 1e9
            
            # Wait for half a second (0.5s) upon entering the tunnel
            if elapsed_time > 0.5:
                # Ignore invalid measurements (out of range -1.0)
                if msg.range > 0.0:
                    total_height = msg.range + self.sensor_height_from_ground
                    
                    # Record the highest value
                    if total_height > self.max_tunnel_height:
                        self.max_tunnel_height = total_height
                        
                    # Publish the currently measured tunnel height
                    height_msg = Float32()
                    height_msg.data = total_height
                    self.height_pub.publish(height_msg)

    def get_avg_distance(self, msg, target_angle_deg):
        target_rad = math.radians(target_angle_deg)
        window_rad = math.radians(self.window_deg)
        
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            # Filter out invalid values (inf, nan, out of range)
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            # Normalize the angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            # Calculate the difference to the target angle
            diff = abs(math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad)))
            
            if diff <= window_rad / 2.0:
                valid_ranges.append(r)
                
        if len(valid_ranges) == 0:
            return 6.0 # Assume safe distance if no valid data
        return sum(valid_ranges) / len(valid_ranges)

    def scan_callback(self, msg):
        # Front obstacle detection (20 cm)
        front_dist = self.get_avg_distance(msg, 0.0)
        if front_dist < 0.20:
            self.get_logger().warn(f'OBSTACLE AHEAD ({front_dist:.2f}m)! Waiting until it is cleared...', throttle_duration_sec=1.0)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return # Pause operation but do not shutdown (resumes when clear)
            
        left_dist = self.get_avg_distance(msg, 90.0)
        right_dist = self.get_avg_distance(msg, -90.0)
        
        twist = Twist()
        
        if self.state == 'APPROACHING':
            # Check if we have entered the tunnel
            if left_dist < self.tunnel_detection_threshold and right_dist < self.tunnel_detection_threshold:
                self.get_logger().info(f'Tunnel detected! Left: {left_dist:.2f}m, Right: {right_dist:.2f}m. Switching to IN_TUNNEL state.')
                self.state = 'IN_TUNNEL'
                self.tunnel_start_time = self.get_clock().now() # Record the tunnel entry time
            else:
                self.get_logger().info(f'Searching for tunnel (Lidar Left: {left_dist:.2f}m, Right: {right_dist:.2f}m)...', throttle_duration_sec=1.0)
                twist.linear.x = self.forward_speed_approach
                
        if self.state == 'IN_TUNNEL':
            # Check if we exited the tunnel
            if left_dist >= self.tunnel_detection_threshold and right_dist >= self.tunnel_detection_threshold:
                self.get_logger().info(f'Tunnel completed! Task finished. Measured Maximum Tunnel Height: {self.max_tunnel_height:.2f} meters.')
                self.state = 'COMPLETED'
            else:
                # Emergency Avoidance (Safety) Check
                if left_dist < self.safety_threshold:
                    self.get_logger().warn(f'EMERGENCY: Too close to left wall ({left_dist:.2f}m)! Evading right.')
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.8 # Sharp right
                elif right_dist < self.safety_threshold:
                    self.get_logger().warn(f'EMERGENCY: Too close to right wall ({right_dist:.2f}m)! Evading left.')
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.8 # Sharp left
                else:
                    self.get_logger().info('Centering and driving forward through the tunnel...', throttle_duration_sec=2.0)
                    
                    # Proportional (P) and Derivative (D) Control for centering
                    error = left_dist - right_dist
                    derivative = error - self.previous_error
                    
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = (self.kp * error) + (self.kd * derivative)
                    
                    # Store current error in memory for the next loop
                    self.previous_error = error
                    
        if self.state == 'COMPLETED':
            # Stop when task is completed
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TunnelNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the motors on shutdown
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
