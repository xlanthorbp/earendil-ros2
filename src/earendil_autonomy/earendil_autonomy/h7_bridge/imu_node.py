"""Convert STM32H723 MPU telemetry records to standard ROS 2 messages."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import String

from .telemetry_parser import as_int, has_valid_int_fields, parse_record


G_MPS2 = 9.80665
MDPS_TO_RAD_S = math.pi / 180000.0


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_node")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("pair_max_age_ms", 500)
        self.declare_parameter("log_rejected_samples", False)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._pair_max_age_ns = int(self.get_parameter("pair_max_age_ms").value) * 1_000_000
        self._log_rejected_samples = bool(
            self.get_parameter("log_rejected_samples").value
        )
        self._gyro: tuple[float, float, float] | None = None
        self._accel: tuple[float, float, float] | None = None
        self._temperature_c: float | None = None
        self._gyro_stamp_ns = 0
        self._accel_stamp_ns = 0

        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 20)
        self._temperature_pub = self.create_publisher(
            Temperature, "imu/temperature", 10
        )
        self.create_subscription(String, "h7/rx_line", self._line_callback, 100)

    def _line_callback(self, msg: String) -> None:
        fields = parse_record(msg.data, "MPU_GYRO,")
        if fields is not None:
            if not self._valid_sample(fields, ("GX", "GY", "GZ", "TC", "OK")):
                self._gyro = None
                self._gyro_stamp_ns = 0
                self._log_rejection("MPU_GYRO")
                return
            self._gyro = (
                as_int(fields, "GX") * MDPS_TO_RAD_S,
                as_int(fields, "GY") * MDPS_TO_RAD_S,
                as_int(fields, "GZ") * MDPS_TO_RAD_S,
            )
            self._temperature_c = as_int(fields, "TC") / 100.0
            self._gyro_stamp_ns = self.get_clock().now().nanoseconds
            self._publish_if_ready()
            return

        fields = parse_record(msg.data, "MPU_ACCEL,")
        if fields is None:
            return
        if not self._valid_sample(fields, ("AX", "AY", "AZ", "TC", "OK")):
            self._accel = None
            self._accel_stamp_ns = 0
            self._log_rejection("MPU_ACCEL")
            return
        self._accel = (
            as_int(fields, "AX") * G_MPS2 / 1000.0,
            as_int(fields, "AY") * G_MPS2 / 1000.0,
            as_int(fields, "AZ") * G_MPS2 / 1000.0,
        )
        self._temperature_c = as_int(fields, "TC") / 100.0
        self._accel_stamp_ns = self.get_clock().now().nanoseconds
        self._publish_if_ready()

    @staticmethod
    def _valid_sample(fields: dict[str, str], required: tuple[str, ...]) -> bool:
        return has_valid_int_fields(fields, required) and as_int(fields, "OK") == 1

    def _log_rejection(self, record_name: str) -> None:
        if self._log_rejected_samples:
            self.get_logger().warn(
                f"Invalid {record_name} record dropped",
                throttle_duration_sec=10.0,
            )

    def _publish_if_ready(self) -> None:
        if self._gyro is None or self._accel is None:
            return
        if abs(self._gyro_stamp_ns - self._accel_stamp_ns) > self._pair_max_age_ns:
            return

        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame_id
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self._gyro
        (
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ) = self._accel
        self._imu_pub.publish(imu)

        if self._temperature_c is not None:
            temperature = Temperature()
            temperature.header.stamp = stamp
            temperature.header.frame_id = self._frame_id
            temperature.temperature = self._temperature_c
            temperature.variance = 0.0
            self._temperature_pub.publish(temperature)

        self._gyro = None
        self._accel = None
        self._gyro_stamp_ns = 0
        self._accel_stamp_ns = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
