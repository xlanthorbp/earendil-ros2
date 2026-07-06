#!/usr/bin/env python3
"""
Hardware Bridge Node for Earendil Bot
Handles Serial communication for Encoders, IMU, Magnetometer and Motors.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        # ---------------------------------------------------------
        # Parameters (Loaded from hardware_params.yaml)
        # ---------------------------------------------------------
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('min_pwm', 60)
        self.declare_parameter('max_pwm', 255)
        
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('ticks_per_rev', 341.2)
        self.declare_parameter('heading_offset', 0.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.heading_offset = self.get_parameter('heading_offset').value

        # ---------------------------------------------------------
        # State Variables
        # ---------------------------------------------------------
        self.last_cmd = None
        self.last_cmd_time = time.time()
        self.serial_buffer = ""
        self.buffer_lock = threading.Lock()
        
        # Sensor Watchdog State
        self.last_enc_time = 0.0
        self.last_mag_time = 0.0
        self.last_imu_time = 0.0
        self.enc_active = False
        self.mag_active = False
        self.imu_active = False
        
        # Odometry state
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # IMU state for merging
        self.latest_yaw = 0.0
        self.has_mag_data = False

        # ---------------------------------------------------------
        # Serial Connection
        # ---------------------------------------------------------
        self.ser = None
        self.serial_lock = threading.Lock()
        self._connect_serial()

        # ---------------------------------------------------------
        # ROS 2 Publishers & Subscribers
        # ---------------------------------------------------------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mag_pub = self.create_publisher(Float32, '/mag/heading', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.raw_pub = self.create_publisher(String, '/arduino/raw_line', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------------------------------------------------
        # Threads and Timers
        # ---------------------------------------------------------
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
        
        # Watchdog for motors
        self.create_timer(0.2, self._keepalive)
        
        # Watchdog for sensors
        self.create_timer(2.0, self._sensor_watchdog)

        self.get_logger().info('Hardware Bridge Node Started Successfully.')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

    # ==================================================
    # MOTOR COMMANDS (Python -> Arduino)
    # ==================================================
    def _cmd_callback(self, msg: Twist):
        if not self.ser:
            return

        v = msg.linear.x
        w = msg.angular.z

        # Simple Motor Mixing (If your Arduino expects FWD/BACK/RIGHT/LEFT)
        # Note: If your teammate writes a custom PID, they might just want raw V and W.
        # Here we keep the old style for compatibility:
        if abs(w) > 0.1:
            pwm = self._velocity_to_pwm(abs(w), max_vel=1.0)
            cmd = f"MOTOR:RIGHT:{pwm}" if w < 0 else f"MOTOR:LEFT:{pwm}"
        elif abs(v) > 0.05:
            pwm = self._velocity_to_pwm(abs(v), max_vel=1.0)
            cmd = f"MOTOR:FWD:{pwm}" if v > 0 else f"MOTOR:BACK:{pwm}"
        else:
            cmd = "MOTOR:STOP"

        if cmd != self.last_cmd:
            self._send_raw(cmd)
            self.last_cmd = cmd
            self.last_cmd_time = time.time()
            self.get_logger().info(f"Command Sent: {cmd}")
        else:
            self.last_cmd_time = time.time()

    def _velocity_to_pwm(self, vel, max_vel=1.0):
        vel = min(abs(vel), max_vel)
        pwm = self.min_pwm + (self.max_pwm - self.min_pwm) * (vel / max_vel)
        return int(pwm)

    def _keepalive(self):
        if self.last_cmd and self.last_cmd != "MOTOR:STOP":
            if time.time() - self.last_cmd_time > 1.0:
                self.last_cmd = "MOTOR:STOP"
                if self.ser:
                    self._send_raw(self.last_cmd)
                self.get_logger().warn("Watchdog triggered! Stopping motors.")
            elif self.ser:
                self._send_raw(self.last_cmd)

    def _sensor_watchdog(self):
        current_time = time.time()
        
        # Check Encoder
        if self.enc_active and (current_time - self.last_enc_time > 2.0):
            self.enc_active = False
            self.get_logger().warn("Encoder data lost! Continuing with available sensors...")
            
        # Check Magnetometer
        if self.mag_active and (current_time - self.last_mag_time > 2.0):
            self.mag_active = False
            self.get_logger().warn("Magnetometer data lost! Continuing with available sensors...")
            
        # Check IMU
        if self.imu_active and (current_time - self.last_imu_time > 2.0):
            self.imu_active = False
            self.get_logger().warn("IMU data lost! Continuing with available sensors...")

    def _send_raw(self, cmd):
        try:
            with self.serial_lock:
                self.ser.write((cmd + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    # ==================================================
    # SERIAL READER (Arduino -> Python)
    # ==================================================
    def _serial_reader(self):
        while rclpy.ok():
            if not self.ser:
                time.sleep(1.0)
                continue
                
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    with self.serial_lock:
                        chunk = self.ser.read(waiting).decode('ascii', errors='ignore')
                    
                    lines_to_process = []
                    with self.buffer_lock:
                        self.serial_buffer += chunk
                        
                        while '\n' in self.serial_buffer:
                            line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                            lines_to_process.append(line.strip())
                            
                    for line in lines_to_process:
                        if not line:
                            continue

                        # Publish raw string for debugging
                        raw_msg = String()
                        raw_msg.data = line
                        self.raw_pub.publish(raw_msg)

                        # Parse based on Prefix
                        if line.startswith("ENC,"):
                            self._parse_encoder(line)
                        elif line.startswith("MAG,"):
                            self._parse_mag(line)
                        elif line.startswith("IMU,"):
                            self._parse_imu(line)
                        elif line.startswith("WARN,"):
                            self.get_logger().warn(f"Arduino: {line}")
                        elif line.startswith("ERR,"):
                            self.get_logger().error(f"Arduino: {line}")
                else:
                    time.sleep(0.01)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1.0)
            except Exception:
                pass

    # ==================================================
    # SENSOR PARSERS
    # ==================================================
    def _parse_encoder(self, line):
        # Expected Format: ENC,left_ticks,right_ticks,dt_ms
        self.last_enc_time = time.time()
        if not self.enc_active:
            self.enc_active = True
            self.get_logger().info("Encoder data stream established/restored.")

        try:
            parts = line.split(',')
            if len(parts) < 4:
                return
                
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
            dt_ms = float(parts[3])
            
            # If first reading, just initialize
            if self.prev_left_ticks is None:
                self.prev_left_ticks = left_ticks
                self.prev_right_ticks = right_ticks
                return
                
            delta_left = left_ticks - self.prev_left_ticks
            delta_right = right_ticks - self.prev_right_ticks
            
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            
            dt = dt_ms / 1000.0
            if dt <= 0:
                return
                
            # Kinematics calculations
            distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            dist_left = delta_left * distance_per_tick
            dist_right = delta_right * distance_per_tick
            
            dist_center = (dist_right + dist_left) / 2.0
            delta_theta = (dist_right - dist_left) / self.wheel_separation
            
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
            self.theta += delta_theta
            
            linear_vel = dist_center / dt
            angular_vel = delta_theta / dt
            
            # Publish Odometry
            current_time = self.get_clock().now().to_msg()
            
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            
            odom.twist.twist.linear.x = linear_vel
            odom.twist.twist.angular.z = angular_vel
            
            self.odom_pub.publish(odom)
            
            # Publish TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            self.tf_broadcaster.sendTransform(t)
            
        except ValueError:
            pass

    def _parse_mag(self, line):
        # Expected Format: MAG,time_ms,heading,...
        self.last_mag_time = time.time()
        if not self.mag_active:
            self.mag_active = True
            self.get_logger().info("Magnetometer data stream established/restored.")

        try:
            parts = line.split(',')
            if len(parts) < 3:
                return

            heading_deg = float(parts[2]) + self.heading_offset
            heading_deg = (heading_deg + 360.0) % 360.0

            # Publish raw heading (degrees)
            h_msg = Float32()
            h_msg.data = heading_deg
            self.mag_pub.publish(h_msg)

            # Save latest yaw for IMU merging
            self.latest_yaw = math.radians(heading_deg)
            self.has_mag_data = True

        except (ValueError, IndexError):
            pass
            
    def _parse_imu(self, line):
        # Expected Format: IMU,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
        self.last_imu_time = time.time()
        if not self.imu_active:
            self.imu_active = True
            self.get_logger().info("IMU data stream established/restored.")

        try:
            parts = line.split(',')
            if len(parts) < 7:
                return
                
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            imu_msg.angular_velocity.x = float(parts[1])
            imu_msg.angular_velocity.y = float(parts[2])
            imu_msg.angular_velocity.z = float(parts[3])
            
            imu_msg.linear_acceleration.x = float(parts[4])
            imu_msg.linear_acceleration.y = float(parts[5])
            imu_msg.linear_acceleration.z = float(parts[6])
            
            # Add Covariances
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001
            
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            # Fill Orientation if MAG is available
            if self.has_mag_data:
                imu_msg.orientation.z = math.sin(self.latest_yaw / 2.0)
                imu_msg.orientation.w = math.cos(self.latest_yaw / 2.0)
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01
            else:
                imu_msg.orientation_covariance[0] = -1.0
            
            self.imu_pub.publish(imu_msg)
            self.imu_raw_pub.publish(imu_msg)
            
        except (ValueError, IndexError):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
