"""Collect valid H7 MAG_IMU samples and atomically write calibration JSON."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .magnetometer_math import build_minmax_calibration, valid_mag_sample
from .telemetry_parser import parse_record


class MagnetometerCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("magnetometer_calibrator")
        self.declare_parameter("duration_s", 90.0)
        self.declare_parameter("minimum_samples", 300)
        self.declare_parameter("input_topic", "/earendil/h7/rx_line")
        self.declare_parameter(
            "output_file", "~/.config/earendil/mag_calibration.json"
        )
        self.declare_parameter("minimum_axis_radius", 50.0)
        self.declare_parameter("warn_radius_ratio", 1.60)

        self._duration_s = float(self.get_parameter("duration_s").value)
        self._minimum_samples = int(self.get_parameter("minimum_samples").value)
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_path = Path(
            str(self.get_parameter("output_file").value)
        ).expanduser()
        self._minimum_axis_radius = float(
            self.get_parameter("minimum_axis_radius").value
        )
        self._warn_radius_ratio = float(
            self.get_parameter("warn_radius_ratio").value
        )
        if self._duration_s <= 0.0:
            raise ValueError("duration_s must be greater than zero")
        if self._minimum_samples <= 0:
            raise ValueError("minimum_samples must be greater than zero")

        self.minimum = [math.inf, math.inf, math.inf]
        self.maximum = [-math.inf, -math.inf, -math.inf]
        self.valid_samples = 0
        self.ignored_samples = 0
        self.started_at: float | None = None
        self.done = False
        self.exit_code = 1

        self.create_subscription(String, self._input_topic, self._line_callback, 200)
        self.create_timer(1.0, self._progress)
        self.get_logger().info(
            f"Calibration ready: {self._duration_s:.1f}s, "
            f"minimum {self._minimum_samples} samples, output {self._output_path}, "
            f"topic {self._input_topic}"
        )
        self.get_logger().info(
            "Rotate sensor/rover covering X, Y, Z positive and negative axes. Counter starts on first valid sample."
        )

    def _line_callback(self, msg: String) -> None:
        if self.done:
            return
        fields = parse_record(msg.data, "MAG_IMU,")
        if fields is None:
            return
        sample = valid_mag_sample(fields)
        if sample is None:
            self.ignored_samples += 1
            return
        if self.started_at is None:
            self.started_at = time.monotonic()
            self.get_logger().info("Valid data received; calibration started")
        for index, value in enumerate(sample):
            self.minimum[index] = min(self.minimum[index], value)
            self.maximum[index] = max(self.maximum[index], value)
        self.valid_samples += 1

    def _progress(self) -> None:
        if self.done or self.started_at is None:
            return
        elapsed = time.monotonic() - self.started_at
        remaining = max(0.0, self._duration_s - elapsed)
        self.get_logger().info(
            f"Remaining {remaining:5.1f} s | samples {self.valid_samples} | "
            f"X[{int(self.minimum[0])},{int(self.maximum[0])}] "
            f"Y[{int(self.minimum[1])},{int(self.maximum[1])}] "
            f"Z[{int(self.minimum[2])},{int(self.maximum[2])}]"
        )
        if elapsed >= self._duration_s:
            self.finish()

    def finish(self, interrupted: bool = False) -> None:
        if self.done:
            return
        self.done = True
        if self.valid_samples < self._minimum_samples:
            self.get_logger().error(
                f"Insufficient samples: {self.valid_samples}; required "
                f"{self._minimum_samples}. File not written."
            )
            self.exit_code = 1
            return
        try:
            result = build_minmax_calibration(
                self.minimum,
                self.maximum,
                minimum_axis_radius=self._minimum_axis_radius,
            )
        except ValueError as exc:
            self.get_logger().error(f"Calibration rejected: {exc}")
            self.exit_code = 1
            return

        calibration = {
            "format_version": 1,
            "source": "QMC5883L raw MX/MY/MZ via ROS 2 /earendil/h7/rx_line",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "topic": "/earendil/h7/rx_line",
            "duration_s": (
                0.0 if self.started_at is None else time.monotonic() - self.started_at
            ),
            "interrupted": interrupted,
            "valid_samples": self.valid_samples,
            "ignored_samples": self.ignored_samples,
            "minimum": dict(zip(("x", "y", "z"), map(int, self.minimum))),
            "maximum": dict(zip(("x", "y", "z"), map(int, self.maximum))),
            **result,
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._output_path.with_suffix(self._output_path.suffix + ".tmp")
        try:
            temporary.write_text(json.dumps(calibration, indent=2), encoding="utf-8")
            temporary.replace(self._output_path)
        except OSError as exc:
            self.get_logger().error(f"Failed to write calibration file: {exc}")
            self.exit_code = 1
            return

        self.get_logger().info(f"Calibration saved: {self._output_path}")
        self.get_logger().info(
            "OFFSET "
            f"X={result['offset']['x']:.6f} "
            f"Y={result['offset']['y']:.6f} "
            f"Z={result['offset']['z']:.6f}"
        )
        self.get_logger().info(
            "SCALE  "
            f"X={result['scale']['x']:.6f} "
            f"Y={result['scale']['y']:.6f} "
            f"Z={result['scale']['z']:.6f}"
        )
        ratio = result["radius_ratio"]
        if ratio > self._warn_radius_ratio:
            self.get_logger().warn(
                f"radius_ratio high ({ratio:.3f}); repeat with broader 3D rotations"
            )
        else:
            self.get_logger().info(f"radius_ratio={ratio:.3f} (good)")
        self.exit_code = 0


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MagnetometerCalibrator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            node.finish(interrupted=True)
        elif not node.done:
            node.exit_code = 130
    finally:
        result = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
