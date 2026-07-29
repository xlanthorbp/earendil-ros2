"""Publish per-wheel H7/F411 telemetry and aggregate joint state."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from earendil_interfaces.msg import WheelTelemetry, WheelTelemetryArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .telemetry_parser import as_int, parse_motor_record


MOTOR_ORDER = ("FL", "FR", "RL", "RR")
WHEEL_IDS = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
WHEEL_NAMES = {
    "FL": "front_left",
    "FR": "front_right",
    "RL": "rear_left",
    "RR": "rear_right",
}
DIRECTION_VALUES = {"R": -1, "N": 0, "F": 1}
DIRECTION_LABELS = {"R": "reverse", "N": "neutral", "F": "forward"}


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


@dataclass
class CachedWheel:
    fields: dict[str, str]
    stamp_ns: int


class WheelTelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_telemetry_node")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("stale_timeout_ms", 1000)
        self.declare_parameter(
            "joint_names",
            [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ],
        )

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._stale_timeout_ns = (
            int(self.get_parameter("stale_timeout_ms").value) * 1_000_000
        )
        self._joint_names = list(self.get_parameter("joint_names").value)
        if len(self._joint_names) != 4:
            raise ValueError("joint_names tam olarak dört eleman içermelidir")

        self._cache: dict[str, CachedWheel] = {}
        self._single_pub = self.create_publisher(
            WheelTelemetry, "wheels/telemetry", 50
        )
        self._array_pub = self.create_publisher(
            WheelTelemetryArray, "wheels/states", 20
        )
        self._joint_pub = self.create_publisher(JointState, "joint_states", 20)
        self.create_subscription(String, "h7/rx_line", self._line_callback, 100)

    def _line_callback(self, msg: String) -> None:
        record = parse_motor_record(msg.data)
        if record is None:
            return

        now = self.get_clock().now()
        self._cache[record.motor] = CachedWheel(dict(record.fields), now.nanoseconds)
        single = self._build_wheel(record.motor, now.nanoseconds, now.to_msg())
        self._single_pub.publish(single)
        self._publish_aggregate(now.nanoseconds, now.to_msg())

    def _build_wheel(self, motor: str, now_ns: int, stamp) -> WheelTelemetry:
        msg = WheelTelemetry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.wheel = WHEEL_IDS[motor]
        msg.wheel_name = WHEEL_NAMES[motor]

        cached = self._cache.get(motor)
        if cached is None:
            msg.direction = 0
            msg.direction_label = "unknown"
            msg.valid = False
            msg.fresh = False
            return msg

        fields = cached.fields
        direction_code = fields.get("DIR", "N")
        age_ns = max(0, now_ns - cached.stamp_ns)
        msg.rpm = float(abs(as_int(fields, "RPM")))
        msg.direction = DIRECTION_VALUES[direction_code]
        msg.direction_label = DIRECTION_LABELS[direction_code]
        msg.target_rpm = float(as_int(fields, "T"))
        msg.drive_duty = as_int(fields, "D")
        msg.commutation_phase = clamp(as_int(fields, "APP_PH"), 0, 0xFF)
        msg.speed_mode = as_int(fields, "SP") == 1
        msg.brake_active = as_int(fields, "BRAKE") == 1
        msg.fault_code = clamp(as_int(fields, "FC"), 0, 0xFFFF)
        msg.hall_state = clamp(as_int(fields, "H"), 0, 0xFF)
        msg.target_pwm = as_int(fields, "PWM_SET")
        msg.applied_pwm = as_int(fields, "PWM_ACT")
        msg.dropped_commands = clamp(as_int(fields, "QDROP"), 0, 0xFFFFFFFF)
        msg.received_bytes = clamp(
            as_int(fields, "RXB"), 0, 0xFFFFFFFFFFFFFFFF
        )
        msg.age_ms = min(age_ns // 1_000_000, 0xFFFFFFFF)
        msg.fresh = age_ns <= self._stale_timeout_ns
        msg.valid = True
        return msg

    def _publish_aggregate(self, now_ns: int, stamp) -> None:
        array = WheelTelemetryArray()
        array.header.stamp = stamp
        array.header.frame_id = self._frame_id
        array.wheels = [
            self._build_wheel(motor, now_ns, stamp) for motor in MOTOR_ORDER
        ]
        array.all_fresh = all(wheel.valid and wheel.fresh for wheel in array.wheels)
        self._array_pub.publish(array)

        joint = JointState()
        joint.header.stamp = stamp
        joint.header.frame_id = self._frame_id
        joint.name = self._joint_names
        joint.velocity = []
        for wheel in array.wheels:
            if not wheel.valid or not wheel.fresh:
                joint.velocity.append(float("nan"))
            else:
                signed_rpm = wheel.rpm * wheel.direction
                joint.velocity.append(signed_rpm * 2.0 * math.pi / 60.0)
        self._joint_pub.publish(joint)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
