#!/usr/bin/env python3
"""
GY-91 (MPU9250) IMU Driver for ROS 2
--------------------------------------
Reads accelerometer, gyroscope, and magnetometer from the GY-91 module
over I2C and publishes sensor_msgs/Imu on /imu/data.

Requires: smbus2  (pip install smbus2)
Hardware: GY-91 connected to Raspberry Pi I2C (SDA/SCL)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import struct
import time

try:
    import smbus2 as smbus
except ImportError:
    import smbus


# MPU9250 Registers
MPU9250_ADDR = 0x68
PWR_MGMT_1   = 0x6B
INT_PIN_CFG   = 0x37
ACCEL_XOUT_H  = 0x3B
GYRO_XOUT_H   = 0x43
WHO_AM_I_MPU  = 0x75

# AK8963 Magnetometer Registers (accessed via I2C bypass)
AK8963_ADDR   = 0x0C
AK8963_CNTL1  = 0x0A
AK8963_HXL    = 0x03
AK8963_ST2    = 0x09
AK8963_WHO    = 0x00

# Scale factors
ACCEL_SCALE = 16384.0   # ±2g  -> LSB/g
GYRO_SCALE  = 131.0     # ±250 deg/s -> LSB/(deg/s)


class Gy91ImuNode(Node):
    def __init__(self):
        super().__init__('gy91_imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')

        bus_num = self.get_parameter('i2c_bus').value
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bus = None
        self.mag_available = False

        try:
            self.bus = smbus.SMBus(bus_num)
            self._init_mpu9250()
            self._init_ak8963()
            self.get_logger().info("GY-91 IMU initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GY-91: {e}")
            self.get_logger().error("Check I2C wiring and run: sudo i2cdetect -y 1")

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)

    # ==================================================
    # Hardware Initialization
    # ==================================================
    def _init_mpu9250(self):
        """Wake up MPU9250 and enable I2C bypass for magnetometer access."""
        # Wake up (clear sleep bit)
        self.bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Verify identity
        who = self.bus.read_byte_data(MPU9250_ADDR, WHO_AM_I_MPU)
        self.get_logger().info(f"MPU9250 WHO_AM_I: 0x{who:02X} (expected 0x71 or 0x73)")

        # Enable I2C bypass so we can talk directly to AK8963 magnetometer
        self.bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x02)
        time.sleep(0.05)

    def _init_ak8963(self):
        """Initialize the AK8963 magnetometer in continuous measurement mode."""
        try:
            who = self.bus.read_byte_data(AK8963_ADDR, AK8963_WHO)
            self.get_logger().info(f"AK8963 WHO_AM_I: 0x{who:02X} (expected 0x48)")

            # Power down first
            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x00)
            time.sleep(0.05)

            # Continuous measurement mode 2 (100 Hz), 16-bit output
            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x16)
            time.sleep(0.05)

            self.mag_available = True
            self.get_logger().info("AK8963 magnetometer initialized.")
        except Exception as e:
            self.get_logger().warn(f"Magnetometer init failed: {e}. Heading will be unavailable.")
            self.mag_available = False

    # ==================================================
    # Sensor Reading
    # ==================================================
    def _read_raw_data(self, addr, reg, length=6):
        """Read raw signed 16-bit values from sensor."""
        data = self.bus.read_i2c_block_data(addr, reg, length)
        values = []
        for i in range(0, length, 2):
            val = struct.unpack('>h', bytes(data[i:i+2]))[0]
            values.append(val)
        return values

    def _read_accel(self):
        """Returns (ax, ay, az) in m/s²."""
        raw = self._read_raw_data(MPU9250_ADDR, ACCEL_XOUT_H, 6)
        return [v / ACCEL_SCALE * 9.80665 for v in raw]

    def _read_gyro(self):
        """Returns (gx, gy, gz) in rad/s."""
        raw = self._read_raw_data(MPU9250_ADDR, GYRO_XOUT_H, 6)
        return [math.radians(v / GYRO_SCALE) for v in raw]

    def _read_mag(self):
        """Returns (mx, my, mz) in raw units, or None if unavailable."""
        if not self.mag_available:
            return None
        try:
            # AK8963 data is little-endian (unlike MPU9250)
            data = self.bus.read_i2c_block_data(AK8963_ADDR, AK8963_HXL, 7)
            # 7th byte (ST2) must be read to signal data read complete
            mx = struct.unpack('<h', bytes(data[0:2]))[0]
            my = struct.unpack('<h', bytes(data[2:4]))[0]
            mz = struct.unpack('<h', bytes(data[4:6]))[0]
            return [mx, my, mz]
        except Exception:
            return None

    # ==================================================
    # Heading Calculation
    # ==================================================
    def _mag_to_quaternion(self, mag):
        """Convert magnetometer XY to a yaw-only quaternion (rotation around Z)."""
        if mag is None:
            return None
        mx, my, _ = mag
        # Heading: 0 = North (positive X), positive = East
        yaw = math.atan2(my, mx)
        # Convert yaw angle to quaternion (only Z-axis rotation)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        return (0.0, 0.0, qz, qw)  # (x, y, z, w)

    # ==================================================
    # Publish Loop
    # ==================================================
    def publish_imu(self):
        if self.bus is None:
            return

        try:
            accel = self._read_accel()
            gyro = self._read_gyro()
            mag = self._read_mag()
        except Exception as e:
            self.get_logger().warn(f"I2C read error: {e}", throttle_duration_sec=5.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation from magnetometer
        quat = self._mag_to_quaternion(mag)
        if quat is not None:
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
        else:
            # Orientation unknown
            msg.orientation_covariance[0] = -1.0

        # Angular velocity (gyroscope)
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        # Linear acceleration (accelerometer)
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Gy91ImuNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
