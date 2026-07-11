import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Bool
import math

class TunnelNavigatorNode(Node):
    def __init__(self):
        super().__init__('tunnel_navigator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.height_pub = self.create_publisher(Float32, '/current_tunnel_height', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.top_sonar_sub = self.create_subscription(Range, '/ir_top', self.top_sonar_callback, 10)
        
        self.aruco_midpoint_sub = self.create_subscription(Point, '/aruco_midpoint', self.aruco_midpoint_callback, 10)
        self.aruco_visible_sub = self.create_subscription(Bool, '/aruco_visible', self.aruco_visible_callback, 10)
        
        # State Variables
        self.state = 'SEARCHING_ENTRANCE'
        self.max_tunnel_height = 0.0
        self.ir_available = False
        self.tunnel_start_time = None
        self.exit_start_time = None
        self.previous_error = 0.0
        
        # Aruco Variables
        self.aruco_visible = False
        self.aruco_angle = 0.0
        self.aruco_distance = 0.0
        
        # Parameters
        self.declare_parameter('forward_speed_approach', 0.2)
        self.declare_parameter('forward_speed_tunnel', 0.3)
        self.declare_parameter('safety_speed', 0.1)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('kp_aruco', 0.02) # P controller for ArUco angle (degrees to rad/s)
        self.declare_parameter('safety_threshold', 0.1)
        self.declare_parameter('tunnel_detection_threshold', 0.3)
        self.declare_parameter('sensor_height_from_ground', 1.0)
        self.declare_parameter('window_deg', 10.0)
        
        self.forward_speed_approach = self.get_parameter('forward_speed_approach').value
        self.forward_speed_tunnel = self.get_parameter('forward_speed_tunnel').value
        self.safety_speed = self.get_parameter('safety_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.kp_aruco = self.get_parameter('kp_aruco').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.tunnel_detection_threshold = self.get_parameter('tunnel_detection_threshold').value
        self.sensor_height_from_ground = self.get_parameter('sensor_height_from_ground').value
        self.window_deg = self.get_parameter('window_deg').value
        
        self.get_logger().info('Tunnel Navigation Node Started.')
        self.get_logger().info('State: SEARCHING_ENTRANCE (Turning right, looking for ArUco)')

    def aruco_visible_callback(self, msg):
        self.aruco_visible = msg.data

    def aruco_midpoint_callback(self, msg):
        # msg.x is angle_x_deg, msg.z is distance
        self.aruco_angle = msg.x
        self.aruco_distance = msg.z

    def top_sonar_callback(self, msg):
        if self.state in ['IN_TUNNEL', 'FOLLOWING_EXIT_ARUCO'] and self.tunnel_start_time is not None:
            elapsed_time = (self.get_clock().now() - self.tunnel_start_time).nanoseconds / 1e9
            if elapsed_time > 0.5:
                if msg.range > 0.0:
                    self.ir_available = True
                    total_height = msg.range + self.sensor_height_from_ground
                    if total_height > self.max_tunnel_height:
                        self.max_tunnel_height = total_height
                    height_msg = Float32()
                    height_msg.data = total_height
                    self.height_pub.publish(height_msg)

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
        if front_dist < 0.20 and self.state != 'COMPLETED':
            self.get_logger().warn(f'OBSTACLE AHEAD ({front_dist:.2f}m)! Waiting...', throttle_duration_sec=1.0)
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
            
        left_dist = self.get_avg_distance(msg, 90.0)
        right_dist = self.get_avg_distance(msg, -90.0)
        
        twist = Twist()
        
        if self.state == 'SEARCHING_ENTRANCE':
            if self.aruco_visible:
                self.get_logger().info('Entrance ArUco detected! Switching to ALIGNING_TO_ENTRANCE.')
                self.state = 'ALIGNING_TO_ENTRANCE'
            else:
                self.get_logger().info('Searching for entrance ArUco...', throttle_duration_sec=2.0)
                twist.angular.z = -0.7 # Turn right (~40 deg/s)
                
        elif self.state == 'ALIGNING_TO_ENTRANCE':
            if left_dist < self.tunnel_detection_threshold and right_dist < self.tunnel_detection_threshold:
                self.get_logger().info(f'Tunnel walls detected! Left: {left_dist:.2f}m, Right: {right_dist:.2f}m. Switching to IN_TUNNEL.')
                self.state = 'IN_TUNNEL'
                self.tunnel_start_time = self.get_clock().now()
            elif self.aruco_visible:
                self.get_logger().info(f'Aligning to entrance... Angle: {self.aruco_angle:.1f}deg', throttle_duration_sec=1.0)
                twist.linear.x = self.forward_speed_approach
                # P controller to align angle to 0. Negate if necessary depending on convention.
                twist.angular.z = -self.kp_aruco * self.aruco_angle 
            else:
                self.get_logger().warn('Lost ArUco during alignment! Reverting to SEARCHING.', throttle_duration_sec=1.0)
                self.state = 'SEARCHING_ENTRANCE'
                
        elif self.state == 'IN_TUNNEL':
            if self.aruco_visible:
                self.get_logger().info('Exit ArUco detected! Switching to FOLLOWING_EXIT_ARUCO.')
                self.state = 'FOLLOWING_EXIT_ARUCO'
            elif left_dist >= self.tunnel_detection_threshold and right_dist >= self.tunnel_detection_threshold:
                self.get_logger().info('Tunnel walls ended. Switching to EXITING_FORWARD_1S.')
                self.state = 'EXITING_FORWARD_1S'
                self.exit_start_time = self.get_clock().now()
            else:
                if left_dist < self.safety_threshold:
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.8
                elif right_dist < self.safety_threshold:
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.8
                else:
                    self.get_logger().info('Centering with Lidar...', throttle_duration_sec=2.0)
                    error = left_dist - right_dist
                    derivative = error - self.previous_error
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = (self.kp * error) + (self.kd * derivative)
                    self.previous_error = error
                    
        elif self.state == 'FOLLOWING_EXIT_ARUCO':
            if not self.aruco_visible:
                self.get_logger().info('Passed exit ArUcos. Switching to EXITING_FORWARD_1S.')
                self.state = 'EXITING_FORWARD_1S'
                self.exit_start_time = self.get_clock().now()
            else:
                self.get_logger().info(f'Following exit ArUco... Angle: {self.aruco_angle:.1f}deg', throttle_duration_sec=1.0)
                twist.linear.x = self.forward_speed_tunnel
                twist.angular.z = -self.kp_aruco * self.aruco_angle
                
        elif self.state == 'EXITING_FORWARD_1S':
            elapsed_time = (self.get_clock().now() - self.exit_start_time).nanoseconds / 1e9
            if elapsed_time < 1.0:
                self.get_logger().info(f'Exiting forward... ({elapsed_time:.1f}/1.0s)', throttle_duration_sec=0.5)
                twist.linear.x = self.forward_speed_tunnel
                twist.angular.z = 0.0
            else:
                if self.ir_available:
                    self.get_logger().info(f'Task finished! Max Tunnel Height: {self.max_tunnel_height:.2f} m.')
                else:
                    self.get_logger().info('Task finished! Height could not be measured.')
                self.state = 'COMPLETED'
                
        elif self.state == 'COMPLETED':
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
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
