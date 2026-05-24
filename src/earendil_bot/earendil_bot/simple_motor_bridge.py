#!/usr/bin/env python3
"""
Simple Motor Bridge + IMU Reader (Unified Serial Node)
--------------------------------------------------------
Single node that handles both:
  - Sending motor commands (cmd_vel -> Arduino string commands)
  - Reading IMU data from Arduino and publishing to /imu/data

Arduino sends:  IMU,heading,ax,ay,az,gx,gy,gz\n
Pi sends:       ileri_yavas\n, dur\n, etc.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import math
import threading


class SimpleMotorBridge(Node):
    def __init__(self):
        super().__init__('simple_motor_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.last_cmd = None
        self.serial_lock = threading.Lock()

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

        # Motor command subscriber
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Start serial reader thread (reads IMU data from Arduino)
        if self.ser:
            self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self._reader_thread.start()

    # ==================================================
    # Motor Command: Twist -> String Command -> Arduino
    # ==================================================
    def cmd_cb(self, msg: Twist):
        if not self.ser:
            return

        v = msg.linear.x
        w = msg.angular.z

        cmd = "dur"

        # Prioritize rotation
        if abs(w) > 0.2:
            if w > 0:
                cmd = "sol_hizli" if abs(w) >= 0.8 else "sol_yavas"
            else:
                cmd = "sag_hizli" if abs(w) >= 0.8 else "sag_yavas"
        elif abs(v) > 0.1:
            if v > 0:
                cmd = "ileri_hizli" if abs(v) >= 0.6 else "ileri_yavas"
            else:
                cmd = "geri_hizli" if abs(v) >= 0.6 else "geri_yavas"
        else:
            cmd = "dur"

        # Only send if command changed
        if cmd != self.last_cmd:
            with self.serial_lock:
                self.ser.write((cmd + "\n").encode('utf-8'))
            self.last_cmd = cmd
            self.get_logger().info(f"Motor: {cmd}")

    # ==================================================
    # Serial Reader: Arduino -> IMU Data -> /imu/data
    # ==================================================
    def _serial_reader(self):
        """Background thread that reads IMU lines from Arduino."""
        while rclpy.ok():
            try:
                with self.serial_lock:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    else:
                        line = None

                if line and line.startswith("IMU,"):
                    self._parse_and_publish_imu(line)

            except Exception:
                pass

    def _parse_and_publish_imu(self, line):
        """Parse: IMU,heading,ax,ay,az,gx,gy,gz"""
        try:
            parts = line.split(',')
            if len(parts) < 8:
                return

            heading_deg = float(parts[1])
            ax = float(parts[2]) * 9.80665  # g -> m/s²
            ay = float(parts[3]) * 9.80665
            az = float(parts[4]) * 9.80665
            gx = math.radians(float(parts[5]))  # dps -> rad/s
            gy = math.radians(float(parts[6]))
            gz = math.radians(float(parts[7]))

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Orientation from magnetometer heading
            if heading_deg >= 0:
                yaw = math.radians(heading_deg)
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = math.sin(yaw / 2.0)
                msg.orientation.w = math.cos(yaw / 2.0)
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01
            else:
                msg.orientation_covariance[0] = -1.0

            # Angular velocity
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # Linear acceleration
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            self.imu_pub.publish(msg)

        except (ValueError, IndexError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorBridge()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
