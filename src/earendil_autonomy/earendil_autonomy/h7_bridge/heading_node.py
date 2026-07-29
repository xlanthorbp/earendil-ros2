"""Publish calibrated, circularly-filtered compass heading from H7 MAG_IMU."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool, Float64, String

from .magnetometer_math import (
    RAW_QMC_LSB_TO_TESLA,
    CircularFilter,
    calculate_heading,
    load_calibration,
    valid_mag_sample,
)
from .telemetry_parser import parse_record


class HeadingNode(Node):
    def __init__(self) -> None:
        super().__init__("heading_node")
        self.declare_parameter(
            "calibration_file", "~/.config/earendil/mag_calibration.json"
        )
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("heading_offset_deg", 0.0)
        self.declare_parameter("filter_alpha", 0.20)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("minimum_horizontal", 5.0)
        self.declare_parameter("heading_timeout_s", 0.75)

        self._calibration_path = Path(
            str(self.get_parameter("calibration_file").value)
        ).expanduser()
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._offset_deg = float(self.get_parameter("heading_offset_deg").value)
        self._invert_x = bool(self.get_parameter("invert_x").value)
        self._invert_y = bool(self.get_parameter("invert_y").value)
        self._swap_xy = bool(self.get_parameter("swap_xy").value)
        self._minimum_horizontal = float(
            self.get_parameter("minimum_horizontal").value
        )
        self._heading_timeout_ns = int(
            float(self.get_parameter("heading_timeout_s").value) * 1.0e9
        )
        alpha = float(self.get_parameter("filter_alpha").value)
        self._filter = CircularFilter(alpha)

        self._calibration = None
        self._calibration_mtime_ns: int | None = None
        self._calibration_wait_logged = False
        self._last_sample_ns = 0
        self._valid_state: bool | None = None

        self._heading_pub = self.create_publisher(Float64, "heading/deg", 20)
        self._raw_heading_pub = self.create_publisher(
            Float64, "heading/raw_deg", 20
        )
        self._field_pub = self.create_publisher(
            MagneticField, "imu/magnetic_field_calibrated", 20
        )
        valid_qos = QoSProfile(depth=1)
        valid_qos.reliability = ReliabilityPolicy.RELIABLE
        valid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._valid_pub = self.create_publisher(Bool, "heading/valid", valid_qos)

        self.create_subscription(String, "h7/rx_line", self._line_callback, 100)
        self.create_timer(1.0, self._reload_calibration)
        self.create_timer(0.1, self._check_freshness)
        self._reload_calibration(force=True)

    def _set_valid(self, value: bool) -> None:
        if self._valid_state == value:
            return
        self._valid_state = value
        msg = Bool()
        msg.data = value
        self._valid_pub.publish(msg)

    def _reload_calibration(self, force: bool = False) -> None:
        try:
            mtime_ns = self._calibration_path.stat().st_mtime_ns
        except OSError:
            self._calibration = None
            self._calibration_mtime_ns = None
            self._set_valid(False)
            if not self._calibration_wait_logged:
                self.get_logger().warn(
                    f"Waiting for calibration file: {self._calibration_path}"
                )
                self._calibration_wait_logged = True
            return
        if not force and mtime_ns == self._calibration_mtime_ns:
            return
        try:
            calibration = load_calibration(self._calibration_path)
        except RuntimeError as exc:
            self._calibration = None
            self._calibration_mtime_ns = mtime_ns
            self._set_valid(False)
            self.get_logger().error(str(exc))
            return
        self._calibration = calibration
        self._calibration_mtime_ns = mtime_ns
        self._calibration_wait_logged = False
        self._filter.reset()
        ratio_text = (
            "unknown"
            if calibration.radius_ratio is None
            else f"{calibration.radius_ratio:.3f}"
        )
        self.get_logger().info(
            f"Magnetometer calibration loaded: {self._calibration_path} "
            f"(radius_ratio={ratio_text})"
        )

    def _line_callback(self, msg: String) -> None:
        if self._calibration is None:
            return
        fields = parse_record(msg.data, "MAG_IMU,")
        if fields is None:
            return
        sample = valid_mag_sample(fields)
        if sample is None:
            return

        calibrated = self._calibration.apply(*sample)
        raw_heading = calculate_heading(
            calibrated,
            offset_deg=self._offset_deg,
            invert_x=self._invert_x,
            invert_y=self._invert_y,
            swap_xy=self._swap_xy,
            minimum_horizontal=self._minimum_horizontal,
        )
        if raw_heading is None:
            return
        filtered_heading = self._filter.update(raw_heading)

        raw_msg = Float64()
        raw_msg.data = raw_heading
        self._raw_heading_pub.publish(raw_msg)
        heading_msg = Float64()
        heading_msg.data = filtered_heading
        self._heading_pub.publish(heading_msg)

        field = MagneticField()
        field.header.stamp = self.get_clock().now().to_msg()
        field.header.frame_id = self._frame_id
        field.magnetic_field.x = calibrated[0] * RAW_QMC_LSB_TO_TESLA
        field.magnetic_field.y = calibrated[1] * RAW_QMC_LSB_TO_TESLA
        field.magnetic_field.z = calibrated[2] * RAW_QMC_LSB_TO_TESLA
        self._field_pub.publish(field)

        self._last_sample_ns = self.get_clock().now().nanoseconds
        self._set_valid(True)

    def _check_freshness(self) -> None:
        if self._calibration is None or self._last_sample_ns == 0:
            self._set_valid(False)
            return
        age_ns = self.get_clock().now().nanoseconds - self._last_sample_ns
        if age_ns > self._heading_timeout_ns:
            self._set_valid(False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
