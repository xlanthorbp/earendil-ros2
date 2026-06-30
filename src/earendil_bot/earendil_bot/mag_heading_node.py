#!/usr/bin/env python3
"""
Magnetometer Heading Node
--------------------------
Reads heading from Arduino GY-271 magnetometer over serial.
Arduino sends:  MAG,heading_degrees\n
This node publishes:
  - /mag/heading  (std_msgs/Float32)  — raw heading in degrees
  - /imu/data     (sensor_msgs/Imu)   — heading as quaternion orientation
                                         (compatible with existing nav nodes)

Usage:
  ros2 run earendil_bot mag_heading_node --ros-args \
    -p port:=/dev/ttyACM1 -p baud:=115200
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import serial
import math
import threading


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__('mag_heading_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Publishers
        self.heading_pub = self.create_publisher(Float32, '/mag/heading', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(
                f"Magnetometer connected on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # Start reader thread
        if self.ser:
            self._thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._thread.start()

    def _serial_reader(self):
        """Background thread: reads MAG,heading lines from Arduino."""
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(
                        'ascii', errors='ignore').strip()
                else:
                    continue

                if not line.startswith("MAG,"):
                    continue

                parts = line.split(',')

                # Support both formats:
                #   Old: MAG,heading           (2 fields, heading at index 1)
                #   New: MAG,time_ms,heading,... (13 fields, heading at index 2)
                if len(parts) == 2:
                    value_str = parts[1]
                elif len(parts) >= 3:
                    value_str = parts[2]
                else:
                    continue

                if value_str == "ERR" or value_str == "ERR_MAG_READ_FAIL":
                    self.get_logger().warn(
                        "Magnetometer read error from Arduino",
                        throttle_duration_sec=5.0)
                    continue

                heading_deg = float(value_str)

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
                # No gyro/accel data from magnetometer
                imu_msg.angular_velocity_covariance[0] = -1.0
                imu_msg.linear_acceleration_covariance[0] = -1.0
                self.imu_pub.publish(imu_msg)

            except (ValueError, IndexError):
                pass
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Serial error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
