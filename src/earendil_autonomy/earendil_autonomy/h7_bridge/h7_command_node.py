"""ROS command and operating-mode supervisor for the Earendil STM32H723."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .command_protocol import (
    is_h7_restart,
    is_pc_link_timeout,
    mode_command,
    mode_from_line,
    normalize_mode,
)


class H7CommandNode(Node):
    """Send commands and supervise the requested H7 operating mode."""

    def __init__(self) -> None:
        super().__init__("h7_command_node")

        self.declare_parameter("auto_recover_enabled", True)
        self.declare_parameter("recovery_delay_s", 3.0)
        self.declare_parameter("mode_retry_period_s", 1.0)
        self.declare_parameter("max_mode_retries", 3)
        self.declare_parameter("max_command_length", 128)
        self.declare_parameter("initial_mode", "")

        self._auto_recover_enabled = bool(
            self.get_parameter("auto_recover_enabled").value
        )
        self._recovery_delay = max(
            0.0, float(self.get_parameter("recovery_delay_s").value)
        )
        self._retry_period = max(
            0.2, float(self.get_parameter("mode_retry_period_s").value)
        )
        self._max_retries = max(0, int(self.get_parameter("max_mode_retries").value))
        self._max_command_length = max(
            1, int(self.get_parameter("max_command_length").value)
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._h7_command_pub = self.create_publisher(String, "h7/command", 20)
        self._desired_mode_pub = self.create_publisher(
            String, "control/desired_mode", state_qos
        )
        self._reported_mode_pub = self.create_publisher(
            String, "control/reported_mode", state_qos
        )
        self._autonomous_latched_pub = self.create_publisher(
            Bool, "control/autonomous_latched", state_qos
        )

        self.create_subscription(
            String, "control/mode_request", self._mode_request_callback, 10
        )
        self.create_subscription(
            String, "control/command", self._command_callback, 20
        )
        self.create_subscription(String, "h7/rx_line", self._rx_callback, 100)
        self.create_subscription(Bool, "h7/connected", self._connected_callback, 10)

        self._connected = False
        self._desired_mode = "unknown"
        self._reported_mode = "unknown"
        self._autonomous_latched = False

        self._recovery_pending = False
        self._recovery_deadline: float | None = None
        self._query_deadline: float | None = None
        self._transition_target: str | None = None
        self._transition_retry_deadline: float | None = None
        self._transition_retries_left = 0

        initial_text = str(self.get_parameter("initial_mode").value).strip()
        if initial_text:
            initial_mode = normalize_mode(initial_text)
            if initial_mode is None:
                self.get_logger().error(
                    f"Invalid initial_mode={initial_text!r}; no mode sent at start"
                )
            else:
                self._set_desired_mode(initial_mode)
        else:
            self._publish_state()

        self.create_timer(0.1, self._update)
        self.get_logger().info("H7 Command Node Active.")

    def _publish_string(self, publisher, value: str) -> None:
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def _publish_state(self) -> None:
        self._publish_string(self._desired_mode_pub, self._desired_mode)
        self._publish_string(self._reported_mode_pub, self._reported_mode)
        latch = Bool()
        latch.data = self._autonomous_latched
        self._autonomous_latched_pub.publish(latch)

    def _publish_h7_command(self, command: str) -> bool:
        clean = command.strip()
        if not self._connected:
            return False
        msg = String()
        msg.data = clean
        self._h7_command_pub.publish(msg)
        return True

    def _set_desired_mode(self, mode: str) -> None:
        self._desired_mode = mode
        self._autonomous_latched = mode == "autonomous"
        if not self._autonomous_latched:
            self._cancel_recovery()
        self._publish_state()

    def _cancel_recovery(self) -> None:
        self._recovery_pending = False
        self._recovery_deadline = None

    def _begin_mode_transition(self, mode: str, reason: str) -> None:
        if not self._connected:
            self.get_logger().warn(
                f"H7 not connected; {mode} request saved but not sent"
            )
            return

        self._transition_target = mode
        self._transition_retries_left = self._max_retries
        self._send_mode_transition(reason)

    def _send_mode_transition(self, reason: str) -> None:
        target = self._transition_target
        if target is None or not self._connected:
            return
        self._publish_h7_command(mode_command(target))
        self._transition_retry_deadline = time.monotonic() + self._retry_period
        self._query_deadline = time.monotonic() + 0.25
        self.get_logger().info(
            f"STM32 mode command sent: {target} ({reason})"
        )

    def _mode_request_callback(self, msg: String) -> None:
        requested = normalize_mode(msg.data)
        if requested is None:
            self.get_logger().error(
                "Invalid mode request. Available values: disarm, manual, autonomous"
            )
            return

        self._set_desired_mode(requested)
        self._begin_mode_transition(requested, "ROS mode_request")

    def _command_callback(self, msg: String) -> None:
        clean = msg.data.strip()
        if not clean:
            self.get_logger().warn("Empty STM32 command rejected")
            return
        if "\r" in clean or "\n" in clean:
            self.get_logger().warn("STM32 command with newline rejected")
            return
        if len(clean.encode("utf-8")) > self._max_command_length:
            self.get_logger().warn("STM32 command exceeding length limit rejected")
            return

        requested_mode = normalize_mode(clean)
        if requested_mode is not None:
            self._set_desired_mode(requested_mode)
            self._begin_mode_transition(requested_mode, "ROS command")
            return

        if not self._publish_h7_command(clean):
            self.get_logger().warn(
                "H7 not connected; command dropped without queuing for safety"
            )

    def _connected_callback(self, msg: Bool) -> None:
        connected = bool(msg.data)
        if connected == self._connected:
            return

        was_connected = self._connected
        self._connected = connected

        if not connected:
            self._reported_mode = "unknown"
            self._query_deadline = None
            self._transition_target = None
            self._transition_retry_deadline = None
            if (
                was_connected
                and self._auto_recover_enabled
                and self._autonomous_latched
            ):
                self._recovery_pending = True
                self._recovery_deadline = None
                self.get_logger().warn(
                    "H7 connection lost; autonomous request retained"
                )
            self._publish_state()
            return

        now = time.monotonic()
        self._query_deadline = now + 0.25
        if self._recovery_pending and self._autonomous_latched:
            self._recovery_deadline = now + self._recovery_delay
            self.get_logger().warn(
                f"H7 reconnected; autonomous mode will be restored after {self._recovery_delay:.1f}s"
            )
        elif self._desired_mode != "unknown":
            self._begin_mode_transition(self._desired_mode, "initial connection")

    def _schedule_autonomous_recovery(self, event: str) -> None:
        if not (
            self._auto_recover_enabled
            and self._autonomous_latched
            and self._desired_mode == "autonomous"
        ):
            return
        self._recovery_pending = True
        self._recovery_deadline = (
            time.monotonic() + self._recovery_delay if self._connected else None
        )
        self._transition_target = None
        self._transition_retry_deadline = None

    def _rx_callback(self, msg: String) -> None:
        line = msg.data
        pc_timeout = is_pc_link_timeout(line)
        h7_restart = is_h7_restart(line)
        if pc_timeout or h7_restart:
            self._reported_mode = "disarm"
            self._publish_state()
            event = "PC-link watchdog DISARM" if pc_timeout else "restart"
            self._schedule_autonomous_recovery(event)

        reported = mode_from_line(line)
        if reported is None:
            return

        self._reported_mode = reported
        self._publish_state()

        if reported == self._transition_target:
            self.get_logger().info(f"STM32 mode verified: {reported}")
            self._transition_target = None
            self._transition_retry_deadline = None

        if reported == "autonomous" and self._autonomous_latched:
            self._cancel_recovery()
            # Initialize sensor telemetry streams on H7
            self._publish_h7_command("imu stream on")
            self._publish_h7_command("imu telper 20")
            self._publish_h7_command("mag telper 50")
            self.get_logger().info("H7 sensor telemetry streams initialized (IMU @ 50Hz, MAG @ 20Hz).")

    def _update(self) -> None:
        if not self._connected:
            return

        now = time.monotonic()

        if self._query_deadline is not None and now >= self._query_deadline:
            self._query_deadline = None
            self._publish_h7_command("mode")

        if (
            self._recovery_pending
            and self._recovery_deadline is not None
            and now >= self._recovery_deadline
        ):
            self._recovery_deadline = None
            if self._reported_mode == "autonomous":
                self._cancel_recovery()
            elif self._autonomous_latched and self._desired_mode == "autonomous":
                self._begin_mode_transition("autonomous", "recovery after disconnect")

        if (
            self._transition_target is not None
            and self._transition_retry_deadline is not None
            and now >= self._transition_retry_deadline
        ):
            if self._transition_retries_left <= 0:
                self.get_logger().error(
                    f"STM32 did not confirm {self._transition_target} mode; max retries reached"
                )
                self._transition_target = None
                self._transition_retry_deadline = None
            else:
                self._transition_retries_left -= 1
                self._send_mode_transition("verification retry")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H7CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
