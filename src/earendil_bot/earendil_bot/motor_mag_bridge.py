#!/usr/bin/env python3
"""
Motor + Magnetometer Bridge (Unified Serial Node)
---------------------------------------------------
Single ROS 2 node for the combined magneto+engine.ino Arduino firmware.
Handles both motor commands and magnetometer telemetry over ONE serial port.

Arduino Protocol:
  Pi  -> Arduino:  MOTOR:FWD:150\n, MOTOR:LEFT:80\n, MOTOR:STOP\n, etc.
  Arduino -> Pi:   MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm

Publishes:
  /mag/heading  (std_msgs/Float32)   — heading in degrees (0-360)
  /imu/data     (sensor_msgs/Imu)    — heading as quaternion (nav node compat)

Subscribes:
  /cmd_vel      (geometry_msgs/Twist) — motor velocity commands

Usage:
  ros2 run earendil_bot motor_mag_bridge --ros-args \
    -p port:=/dev/ttyACM0 -p baud:=115200
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu
import serial
import math
import threading
import time


class MotorMagBridge(Node):
    def __init__(self):
        super().__init__('motor_mag_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('min_pwm', 60)    # Minimum PWM for motor to move
        self.declare_parameter('max_pwm', 255)   # Maximum PWM
        self.declare_parameter('heading_offset', 0.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.heading_offset = self.get_parameter('heading_offset').value

        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.serial_lock = threading.Lock()

        # Serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Connected to Arduino on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

        # Motor command subscriber
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        # Magnetometer publishers
        self.heading_pub = self.create_publisher(Float32, '/mag/heading', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.raw_pub = self.create_publisher(String, '/arduino/raw_line', 10)

        # Watchdog keepalive: Arduino stops motors after 700ms without command.
        # Resend the last active command every 500ms to prevent watchdog trigger.
        self.keepalive_timer = self.create_timer(0.5, self._keepalive)

        # Start serial reader thread
        if self.ser:
            self.serial_buffer = ""
            self._thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._thread.start()

    def destroy_node(self):
        """Called when the node is shut down (Ctrl+C)."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"MOTOR:STOP\n")
                self.ser.close()
                self.get_logger().info("Sent MOTOR:STOP before exit.")
            except Exception:
                pass
        super().destroy_node()

    # ==================================================
    # Motor: Twist -> MOTOR:CMD:PWM -> Arduino
    # ==================================================
    def _velocity_to_pwm(self, velocity, max_vel=1.0):
        """Map velocity magnitude to PWM range [min_pwm, max_pwm]."""
        abs_vel = min(abs(velocity), max_vel)
        if abs_vel < 0.05:
            return 0
        pwm = int(self.min_pwm +
                  (abs_vel / max_vel) * (self.max_pwm - self.min_pwm))
        return min(pwm, self.max_pwm)

    def cmd_cb(self, msg: Twist):
        if not self.ser:
            return

        v = msg.linear.x
        w = msg.angular.z

        # Prioritize rotation
        if abs(w) > 0.1:
            pwm = self._velocity_to_pwm(w, max_vel=1.5)
            if w > 0:
                cmd = f"MOTOR:LEFT:{pwm}"
            else:
                cmd = f"MOTOR:RIGHT:{pwm}"
        elif abs(v) > 0.05:
            pwm = self._velocity_to_pwm(v, max_vel=1.0)
            if v > 0:
                cmd = f"MOTOR:FWD:{pwm}"
            else:
                cmd = f"MOTOR:BACK:{pwm}"
        else:
            cmd = "MOTOR:STOP"

        # Only send if command changed
        if cmd != self.last_cmd:
            self._send_raw(cmd)
            self.last_cmd = cmd
            self.last_cmd_time = time.time()
            self.get_logger().info(f"Motor: {cmd}")
        else:
            # Update time even if command hasn't changed to keep watchdog happy
            self.last_cmd_time = time.time()

    def _keepalive(self):
        """Resend last active motor command to prevent Arduino watchdog."""
        if self.last_cmd and self.last_cmd != "MOTOR:STOP":
            if time.time() - self.last_cmd_time > 1.0:
                self.last_cmd = "MOTOR:STOP"
                if self.ser:
                    self._send_raw(self.last_cmd)
                self.get_logger().warn("CmdVel Watchdog triggered! Stopping motors.", throttle_duration_sec=2.0)
            elif self.ser:
                self._send_raw(self.last_cmd)

    def _send_raw(self, cmd):
        """Send a raw string command to Arduino."""
        try:
            with self.serial_lock:
                self.ser.write((cmd + "\n").encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(
                f"Serial write error: {e}", throttle_duration_sec=5.0)

    # ==================================================
    # Serial Reader: Arduino -> MAG Telemetry -> ROS
    # ==================================================
    def _serial_reader(self):
        """Background thread that reads all lines from Arduino robustly."""
        while rclpy.ok():
            try:
                # Sadece veri varsa oku (bloklanmayı önler)
                waiting = self.ser.in_waiting
                if waiting > 0:
                    with self.serial_lock:
                        chunk = self.ser.read(waiting).decode('ascii', errors='ignore')
                    
                    self.serial_buffer += chunk
                    
                    # Tam satırları ayıkla
                    while '\n' in self.serial_buffer:
                        line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                        line = line.strip()
                        
                        if not line:
                            continue

                        # Ham veriyi yayınla (Debug için)
                        raw_msg = String()
                        raw_msg.data = line
                        self.raw_pub.publish(raw_msg)

                        # MAG telemetry line
                        if line.startswith("MAG,"):
                            self._parse_mag_telemetry(line)

                        # Log warnings and errors from Arduino
                        elif line.startswith("WARN,"):
                            self.get_logger().warn(f"Arduino: {line}")
                        elif line.startswith("ERR,"):
                            self.get_logger().error(f"Arduino: {line}")
                else:
                    # CPU'yu yormamak için kısa bir uyku
                    time.sleep(0.01)

            except serial.SerialException as e:
                self.get_logger().error(
                    f"Serial read error: {e}", throttle_duration_sec=5.0)
                time.sleep(1.0)
            except Exception:
                pass

    def _parse_mag_telemetry(self, line):
        """Parse: MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm"""
        try:
            parts = line.split(',')
            if len(parts) < 3:
                return

            heading_deg = float(parts[2]) + self.heading_offset
            heading_deg = (heading_deg + 360.0) % 360.0

            # Publish raw heading (degrees)
            h_msg = Float32()
            h_msg.data = heading_deg
            self.heading_pub.publish(h_msg)

            # Publish as Imu message (quaternion) for nav node compatibility
            yaw = math.radians(heading_deg)
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw / 2.0)
            imu_msg.orientation.w = math.cos(yaw / 2.0)
            imu_msg.orientation_covariance[0] = 0.05
            imu_msg.orientation_covariance[4] = 0.05
            imu_msg.orientation_covariance[8] = 0.05
            # No gyro/accel data from magnetometer only
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu_msg)

        except (ValueError, IndexError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorMagBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
