#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class DummyTwist:
    class Linear:
        x = 0.0
    class Angular:
        z = 0.0
    def __init__(self):
        self.linear = self.Linear()
        self.angular = self.Angular()

class Test5Node(Node):
    def __init__(self):
        super().__init__('test5')
        
        self.cmd_pub = self.create_publisher(String, '/earendil/control/command', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ir_sub = self.create_subscription(Range, '/ir_top', self.ir_callback, 10)
        self.heading_sub = self.create_subscription(Float64, '/earendil/heading/deg', self.heading_callback, 10)
        
        self.aruco_midpoint_sub = self.create_subscription(Point, '/aruco_midpoint', self.aruco_midpoint_callback, 10)
        self.aruco_visible_sub = self.create_subscription(Bool, '/aruco_visible', self.aruco_visible_callback, 10)
        
        # New subscriptions for left and right ArUco tags
        self.aruco_left_sub = self.create_subscription(Point, '/aruco_left', self.aruco_left_callback, 10)
        self.aruco_right_sub = self.create_subscription(Point, '/aruco_right', self.aruco_right_callback, 10)
        self.aruco_left_visible_sub = self.create_subscription(Bool, '/aruco_left_visible', self.aruco_left_visible_callback, 10)
        self.aruco_right_visible_sub = self.create_subscription(Bool, '/aruco_right_visible', self.aruco_right_visible_callback, 10)
        
        # State Variables
        self.previous_error = 0.0
        self.state = 'SEARCHING_ENTRANCE'
        self.tunnel_start_time = None
        self.exit_lost_time = None
        
        # New State Machine Variables for Alignment
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Search variables
        self.accumulated_yaw = 0.0
        self.last_yaw = None
        self.backup_distance = 2.0
        
        # Alignment results
        self.target_turn_angle = 0.0
        self.target_travel_dist = 0.0
        self.target_wall_angle = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        
        # Tunnel Timer State
        self.tunnel_timer_started = False
        self.tunnel_timer_completed = False
        self.tunnel_timer_start_time = None
        self.tunnel_timer_end_time = None
        self.speeds_during_tunnel = []
        self.tunnel_length = 0.0
        self.current_speed = 0.0
        self.out_of_tunnel_consecutive_count = 0
        self.first_out_of_tunnel_time = None
        
        # Aruco Variables
        self.aruco_visible = False
        self.aruco_angle = 0.0
        self.aruco_width_diff = 0.0
        self.aruco_distance = 0.0
        
        # Aruco Variables (Left and Right)
        self.aruco_left_visible = False
        self.aruco_left_angle = 0.0
        self.aruco_left_dist = 0.0
        self.last_aruco_left_time = None
        
        self.aruco_right_visible = False
        self.aruco_right_angle = 0.0
        self.aruco_right_dist = 0.0
        self.last_aruco_right_time = None
        
        # Parameters
        self.declare_parameter('forward_speed_approach', 0.2)
        self.declare_parameter('forward_speed_tunnel', 0.2)
        self.declare_parameter('safety_speed', 0.1)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('safety_threshold', 0.25)
        self.declare_parameter('tunnel_detection_threshold', 1.0)
        self.declare_parameter('window_deg', 10.0)
        self.declare_parameter('kp_aruco', 0.02)
        self.declare_parameter('kp_width', 0.5)
        self.declare_parameter('ir_height', 1.0)
        self.declare_parameter('ir_limit', 1.49)
        self.declare_parameter('kp_turn', 0.8)
        
        self.forward_speed_approach = self.get_parameter('forward_speed_approach').value
        self.forward_speed_tunnel = self.get_parameter('forward_speed_tunnel').value
        self.safety_speed = self.get_parameter('safety_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.tunnel_detection_threshold = self.get_parameter('tunnel_detection_threshold').value
        self.window_deg = self.get_parameter('window_deg').value
        self.kp_aruco = self.get_parameter('kp_aruco').value
        self.kp_width = self.get_parameter('kp_width').value
        self.ir_height = self.get_parameter('ir_height').value
        self.ir_limit = self.get_parameter('ir_limit').value
        self.kp_turn = self.get_parameter('kp_turn').value
        
        # ROS 2 Shutdown Hook: Triggered when the node shuts down (error or normal)
        rclpy.get_default_context().on_shutdown(self.stop_motors_safely)
        
        self.get_logger().info('Test5 Node Started (Entrance and Exit ArUco Enabled).')
        self.get_logger().info('State: SEARCHING_ENTRANCE (Turning right, looking for 2 ArUcos)')

    def aruco_visible_callback(self, msg):
        self.aruco_visible = msg.data

    def aruco_midpoint_callback(self, msg):
        self.aruco_angle = msg.x
        self.aruco_width_diff = msg.y
        self.aruco_distance = msg.z

    def aruco_left_callback(self, msg):
        self.aruco_left_angle = msg.x
        self.aruco_left_dist = msg.z
        self.aruco_left_visible = True
        self.last_aruco_left_time = self.get_clock().now()

    def aruco_right_callback(self, msg):
        self.aruco_right_angle = msg.x
        self.aruco_right_dist = msg.z
        self.aruco_right_visible = True
        self.last_aruco_right_time = self.get_clock().now()

    def aruco_left_visible_callback(self, msg):
        self.aruco_left_visible = msg.data

    def aruco_right_visible_callback(self, msg):
        self.aruco_right_visible = msg.data

    def normalize_angle(self, angle):
        """Wraps the given angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def euler_from_quaternion(self, q):
        """Converts quaternion to roll, pitch, yaw."""
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def stop_motors_safely(self):
        """Sends 0 velocity to motors when the node shuts down for any reason."""
        self.get_logger().info('Shutdown signal received: Stopping motors for safety...')
        self.send_motor_cmd(0.0, 0.0)

    def send_motor_cmd(self, v, w):
        msg = String()
        if abs(w) > 0.05:
            pwm = 60 + int((abs(w) / 0.5) * 30)
            msg.data = f"l {min(90, max(60, pwm))}" if w > 0 else f"r {min(90, max(60, pwm))}"
        elif abs(v) > 0.05:
            pwm = 60 + int((abs(v) / 0.3) * 30)
            msg.data = f"f {min(90, max(60, pwm))}" if v > 0 else f"b {min(90, max(60, pwm))}"
        else:
            msg.data = "stop"
        self.cmd_pub.publish(msg)

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

    def heading_callback(self, msg):
        yaw = math.radians(msg.data)
        if self.odom_received:
            if self.state == 'SEARCHING_ENTRANCE':
                diff = self.normalize_angle(yaw - self.current_yaw)
                self.accumulated_yaw += diff
        else:
            self.odom_received = True
        self.current_yaw = yaw

    def ir_callback(self, msg):
        dist = msg.range
        
        # Check if the ceiling is detected
        # Since the sensor cannot measure above ir_limit, any non -1.0 range reading below the limit means we are under the ceiling
        is_under_ceiling = (dist != -1.0) and (dist <= self.ir_limit)
        
        if is_under_ceiling:
            self.out_of_tunnel_consecutive_count = 0
            self.first_out_of_tunnel_time = None
            if not self.tunnel_timer_started:
                self.tunnel_timer_started = True
                self.tunnel_timer_start_time = self.get_clock().now()
                self.speeds_during_tunnel = []
                if self.current_speed != 0.0:
                    self.speeds_during_tunnel.append(self.current_speed)
                self.get_logger().info(f"Tunnel timer started! Roof detected (Distance: {dist:.2f} m)")
        else:
            if self.tunnel_timer_started and not self.tunnel_timer_completed:
                self.out_of_tunnel_consecutive_count += 1
                if self.first_out_of_tunnel_time is None:
                    self.first_out_of_tunnel_time = self.get_clock().now()
                if self.out_of_tunnel_consecutive_count >= 3:
                    self.tunnel_timer_completed = True
                    self.tunnel_timer_end_time = self.first_out_of_tunnel_time
                    t = (self.tunnel_timer_end_time - self.tunnel_timer_start_time).nanoseconds / 1e9
                    V = sum(self.speeds_during_tunnel) / len(self.speeds_during_tunnel) if self.speeds_during_tunnel else 0.0
                    self.tunnel_length = V * t
                    self.get_logger().info(f"Tunnel timer completed! Duration: {t:.2f}s, Avg Speed: {V:.2f}m/s, Measured Tunnel Length: {self.tunnel_length:.2f}m")

    def scan_callback(self, msg):
        # Watchdog timeout check for left/right ArUco tags
        now = self.get_clock().now()
        if self.last_aruco_left_time is not None:
            if (now - self.last_aruco_left_time).nanoseconds / 1e9 > 1.0:
                self.aruco_left_visible = False
        if self.last_aruco_right_time is not None:
            if (now - self.last_aruco_right_time).nanoseconds / 1e9 > 1.0:
                self.aruco_right_visible = False

        # Fallback simulator / estimator if individual ArUco tags are not visible
        # but the midpoint ArUco is visible:
        if (not self.aruco_left_visible or not self.aruco_right_visible) and self.aruco_visible:
            # Assume a tunnel entrance width of 5.0m (2.5m on each side of the midpoint)
            theta_mid = math.radians(self.aruco_angle)
            x_mid = self.aruco_distance * math.cos(theta_mid)
            y_mid = self.aruco_distance * math.sin(theta_mid)
            
            # Left tag coordinate in robot frame
            x_l = x_mid
            y_l = y_mid + 2.5
            self.aruco_left_dist = math.hypot(x_l, y_l)
            self.aruco_left_angle = math.degrees(math.atan2(y_l, x_l))
            self.aruco_left_visible = True
            
            # Right tag coordinate in robot frame
            x_r = x_mid
            y_r = y_mid - 2.5
            self.aruco_right_dist = math.hypot(x_r, y_r)
            self.aruco_right_angle = math.degrees(math.atan2(y_r, x_r))
            self.aruco_right_visible = True

        front_dist = self.get_avg_distance(msg, 0.0)
        twist = DummyTwist()
        
        if front_dist < 0.20 and self.state not in ['COMPLETED', 'BACKING_UP_FOR_SEARCH']:
            self.get_logger().warn(f'OBSTACLE AHEAD ({front_dist:.2f}m)! Stopping.', throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.send_motor_cmd(twist.linear.x, twist.angular.z)
            return
            
        left_dist = self.get_avg_distance(msg, 90.0)
        right_dist = self.get_avg_distance(msg, -90.0)
        
        if self.state == 'SEARCHING_ENTRANCE':
            if self.aruco_left_visible and self.aruco_right_visible:
                self.get_logger().info('Both entrance ArUco tags detected! Odometry disabled, skipping alignment and approaching directly.')
                v = 0.0
                w = 0.0
                self.state = 'APPROACHING_ENTRANCE'
            else:
                # If we've completed a 360-degree rotation without finding both tags
                if abs(self.accumulated_yaw) >= 2.0 * math.pi - 0.05:
                    self.get_logger().warn('Completed 360-degree rotation but did not detect both ArUcos. Backing up for 5 seconds.')
                    v = 0.0
                    w = 0.0
                    self.backup_start_time = self.get_clock().now()
                    self.state = 'BACKING_UP_FOR_SEARCH'
                else:
                    self.get_logger().info(f'Searching for both entrance ArUcos (Accumulated turn: {math.degrees(self.accumulated_yaw):.1f}deg)...', throttle_duration_sec=2.0)
                    w = -0.4 # Turn right slowly
                    
        elif self.state == 'BACKING_UP_FOR_SEARCH':
            elapsed = (self.get_clock().now() - self.backup_start_time).nanoseconds / 1e9
            if elapsed >= 5.0:
                self.get_logger().info('Backed up 5 seconds. Resuming 360-degree search.')
                v = 0.0
                self.accumulated_yaw = 0.0
                self.state = 'SEARCHING_ENTRANCE'
            else:
                self.get_logger().info(f'Backing up... {elapsed:.1f}s / 5.0s', throttle_duration_sec=1.0)
                v = -self.forward_speed_approach
                w = 0.0
                
        elif self.state == 'CALCULATING_ALIGNMENT':
            # Stop the robot first
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            theta1 = math.radians(self.aruco_left_angle)
            theta2 = math.radians(self.aruco_right_angle)
            
            # Project to local XY
            x1 = self.aruco_left_dist * math.cos(theta1)
            y1 = self.aruco_left_dist * math.sin(theta1)
            x2 = self.aruco_right_dist * math.cos(theta2)
            y2 = self.aruco_right_dist * math.sin(theta2)
            
            D = math.hypot(x2 - x1, y2 - y1)
            if D < 0.1:
                D = 0.1 # Prevent division by zero
                
            vx = x2 - x1
            vy = y2 - y1
            
            # Unit baseline vector
            ux = vx / D
            uy = vy / D
            
            # Projection of robot (0,0) relative to ArUco Left (x1, y1)
            p = -x1 * ux - y1 * uy
            
            # Required translation along baseline to reach midpoint
            y_diff = (D / 2.0) - p
            
            # Translation vector in robot frame
            tx = y_diff * ux
            ty = y_diff * uy
            
            self.target_turn_angle = math.atan2(ty, tx)
            self.target_travel_dist = math.hypot(tx, ty)
            
            # Projection point on baseline (on the wall)
            proj_x = x1 + p * ux
            proj_y = y1 + p * uy
            wall_angle_orig = math.atan2(proj_y, proj_x)
            
            # Angle to face the wall from the new heading (after turning target_turn_angle)
            self.target_wall_angle = self.normalize_angle(wall_angle_orig - self.target_turn_angle)
            
            # Record starting pose for odometry tracking
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            
            self.get_logger().info(f"CALCULATING_ALIGNMENT:")
            self.get_logger().info(f"  ArUco Left: dist={self.aruco_left_dist:.2f}m, angle={self.aruco_left_angle:.1f}deg")
            self.get_logger().info(f"  ArUco Right: dist={self.aruco_right_dist:.2f}m, angle={self.aruco_right_angle:.1f}deg")
            self.get_logger().info(f"  Calculated Tag Distance (D): {D:.2f}m")
            self.get_logger().info(f"  Robot proj on baseline (p): {p:.2f}m")
            self.get_logger().info(f"  y_diff to midpoint: {y_diff:.2f}m")
            self.get_logger().info(f"  Turn parallel: {math.degrees(self.target_turn_angle):.1f}deg")
            self.get_logger().info(f"  Drive: {self.target_travel_dist:.2f}m")
            self.get_logger().info(f"  Turn to face wall: {math.degrees(self.target_wall_angle):.1f}deg")
            
            self.state = 'TURNING_PARALLEL'
            
        elif self.state == 'TURNING_PARALLEL':
            target_yaw = self.normalize_angle(self.start_yaw + self.target_turn_angle)
            error = self.normalize_angle(target_yaw - self.current_yaw)
            
            if abs(error) < math.radians(2.0):
                self.get_logger().info(f"Parallel alignment complete! Current yaw: {math.degrees(self.current_yaw):.1f}deg. Starting drive.")
                twist.angular.z = 0.0
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.state = 'DRIVING_TO_MIDPOINT'
            else:
                angular_speed = self.kp_turn * error
                if angular_speed > 0:
                    angular_speed = max(0.15, min(angular_speed, 0.4))
                else:
                    angular_speed = min(-0.15, max(angular_speed, -0.4))
                twist.linear.x = 0.0
                twist.angular.z = angular_speed
                self.get_logger().info(f"Turning parallel: error={math.degrees(error):.1f}deg, cmd_z={twist.angular.z:.2f}", throttle_duration_sec=0.5)
                
        elif self.state == 'DRIVING_TO_MIDPOINT':
            dist_driven = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
            if dist_driven >= self.target_travel_dist:
                self.get_logger().info(f"Reached midpoint! Distance driven: {dist_driven:.2f}m. Starting rotation to face wall.")
                twist.linear.x = 0.0
                self.start_yaw = self.current_yaw
                self.state = 'TURNING_TO_FACE_WALL'
            else:
                twist.linear.x = self.forward_speed_approach
                twist.angular.z = 0.0
                self.get_logger().info(f"Driving to midpoint: {dist_driven:.2f}m / {self.target_travel_dist:.2f}m", throttle_duration_sec=0.5)
                
        elif self.state == 'TURNING_TO_FACE_WALL':
            target_yaw = self.normalize_angle(self.start_yaw + self.target_wall_angle)
            error = self.normalize_angle(target_yaw - self.current_yaw)
            
            if abs(error) < math.radians(2.0):
                self.get_logger().info("Aligned to wall/entrance! Switching to APPROACHING_ENTRANCE.")
                twist.angular.z = 0.0
                self.state = 'APPROACHING_ENTRANCE'
            else:
                angular_speed = self.kp_turn * error
                if angular_speed > 0:
                    angular_speed = max(0.15, min(angular_speed, 0.4))
                else:
                    angular_speed = min(-0.15, max(angular_speed, -0.4))
                twist.linear.x = 0.0
                twist.angular.z = angular_speed
                self.get_logger().info(f"Turning to face wall: error={math.degrees(error):.1f}deg, cmd_z={twist.angular.z:.2f}", throttle_duration_sec=0.5)
                
        elif self.state == 'APPROACHING_ENTRANCE':
            if left_dist < self.tunnel_detection_threshold and right_dist < self.tunnel_detection_threshold:
                self.get_logger().info(f'---> WALLS DETECTED! (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). ENTERED TUNNEL!')
                self.state = 'IN_TUNNEL'
                self.tunnel_start_time = self.get_clock().now()
            else:
                twist.linear.x = self.forward_speed_approach
                if self.aruco_visible:
                    twist.angular.z = -self.kp_aruco * self.aruco_angle + self.kp_width * self.aruco_width_diff
                    self.get_logger().info(f'Approaching Entrance - Correcting: angle={self.aruco_angle:.1f}deg, width_diff={self.aruco_width_diff:.3f}, cmd_z={twist.angular.z:.2f}', throttle_duration_sec=1.0)
                else:
                    twist.angular.z = 0.0
                
        elif self.state == 'IN_TUNNEL':
            if left_dist >= self.tunnel_detection_threshold and right_dist >= self.tunnel_detection_threshold:
                self.get_logger().info(f'---> WALLS ENDED! (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). EXITED TUNNEL!')
                if self.tunnel_timer_started and not self.tunnel_timer_completed:
                    self.tunnel_timer_completed = True
                    self.tunnel_timer_end_time = self.get_clock().now()
                    t = (self.tunnel_timer_end_time - self.tunnel_timer_start_time).nanoseconds / 1e9
                    V = sum(self.speeds_during_tunnel) / len(self.speeds_during_tunnel) if self.speeds_during_tunnel else 0.0
                    self.tunnel_length = V * t
                    self.get_logger().info(f"Forced stopping timer on tunnel exit! Duration: {t:.2f}s, Avg Speed: {V:.2f}m/s, Length: {self.tunnel_length:.2f}m")
                
                if self.tunnel_timer_completed:
                    self.get_logger().info(f'\n======================================================\n'
                                           f'TUNNEL COMPLETED! MEASURED TUNNEL LENGTH: {self.tunnel_length:.2f} m\n'
                                           f'======================================================\n')
                else:
                    self.get_logger().warn("Tunnel length measurement not completed or timer did not finish!")
                
                self.get_logger().info('Starting search for exit ArUco.')
                self.state = 'SEARCHING_EXIT_ARUCO'
            else:
                if left_dist < self.safety_threshold:
                    self.get_logger().warn('Too close to the left wall! Turning right.', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.15
                elif right_dist < self.safety_threshold:
                    self.get_logger().warn('Too close to the right wall! Turning left.', throttle_duration_sec=1.0)
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.15
                else:
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = 0.0
                    
        elif self.state == 'SEARCHING_EXIT_ARUCO':
            if self.aruco_visible:
                self.get_logger().info('Exit ArUco detected! Switching to ALIGNING_EXIT_ARUCO.')
                self.state = 'ALIGNING_EXIT_ARUCO'
            else:
                self.get_logger().info('Searching for exit ArUco...', throttle_duration_sec=2.0)
                twist.angular.z = -0.5
                
        elif self.state == 'ALIGNING_EXIT_ARUCO':
            if self.aruco_visible:
                if abs(self.aruco_angle) > 3.0:
                    self.get_logger().info(f'Aligning to exit in place... Angle: {self.aruco_angle:.1f}deg', throttle_duration_sec=1.0)
                    twist.linear.x = 0.0
                    angular_speed = -self.kp_aruco * self.aruco_angle
                    if angular_speed > 0 and angular_speed < 0.15:
                        angular_speed = 0.15
                    elif angular_speed < 0 and angular_speed > -0.15:
                        angular_speed = -0.15
                    twist.angular.z = angular_speed
                else:
                    self.get_logger().info('Perfectly aligned to the exit center! Just going straight (APPROACHING_EXIT).')
                    self.state = 'APPROACHING_EXIT'
            else:
                self.get_logger().warn('Exit ArUco lost while aligning! Searching again.', throttle_duration_sec=1.0)
                self.state = 'SEARCHING_EXIT_ARUCO'
                
        elif self.state == 'APPROACHING_EXIT':
            if self.aruco_visible:
                twist.linear.x = self.forward_speed_approach
                twist.angular.z = -self.kp_aruco * self.aruco_angle + self.kp_width * self.aruco_width_diff
                self.get_logger().info(f'Approaching Exit - Correcting: angle={self.aruco_angle:.1f}deg, width_diff={self.aruco_width_diff:.3f}, cmd_z={twist.angular.z:.2f}', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('Exit ArUco lost while approaching! Stopping in 1 second...')
                self.exit_lost_time = self.get_clock().now()
                self.state = 'STOPPING_AT_EXIT'
                
        elif self.state == 'STOPPING_AT_EXIT':
            if self.aruco_visible:
                self.get_logger().info('Exit ArUco visible again! Resuming approach.')
                self.state = 'APPROACHING_EXIT'
            else:
                elapsed_sec = (self.get_clock().now() - self.exit_lost_time).nanoseconds / 1e9
                if elapsed_sec >= 1.0:
                    self.get_logger().info('1 second passed since Aruco was lost. Task Completed!')
                    self.state = 'COMPLETED'
                else:
                    twist.linear.x = self.forward_speed_approach
                    twist.angular.z = 0.0
                    
        elif self.state == 'COMPLETED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        if self.tunnel_timer_completed:
            display_len = self.tunnel_length
        elif self.tunnel_timer_started and self.tunnel_timer_start_time is not None:
            elapsed_t = (self.get_clock().now() - self.tunnel_timer_start_time).nanoseconds / 1e9
            current_avg_v = sum(self.speeds_during_tunnel) / len(self.speeds_during_tunnel) if self.speeds_during_tunnel else 0.0
            display_len = current_avg_v * elapsed_t
        else:
            display_len = 0.0
        
        self.get_logger().info(
            f'[{self.state}] Dist-> L: {left_dist:.2f}m, R: {right_dist:.2f}m, F: {front_dist:.2f}m | '
            f'Tunnel Length: {display_len:.2f}m | '
            f'Motor-> Fwd: {twist.linear.x:.2f}, Turn: {twist.angular.z:.2f}',
            throttle_duration_sec=0.5
        )
            
        self.send_motor_cmd(twist.linear.x, twist.angular.z)

def main(args=None):
    rclpy.init(args=args)
    node = Test5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = String()
        msg.data = "stop"
        node.cmd_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
