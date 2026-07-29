"""Kinematic command mapper converting ROS 2 /cmd_vel Twist messages to STM32H7 terminal commands."""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class H7KinematicsNode(Node):
    """Subscribes to ROS 2 Twist (/cmd_vel) and publishes motion commands to H7."""

    def __init__(self) -> None:
        super().__init__("h7_kinematics_node")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.160)
        self.declare_parameter("max_rpm", 200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_command_topic", "h7/command")
        self.declare_parameter("deadband_linear", 0.01)
        self.declare_parameter("deadband_angular", 0.05)

        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheel_separation = float(self.get_parameter("wheel_separation").value)
        self._max_rpm = int(self.get_parameter("max_rpm").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        out_topic = str(self.get_parameter("output_command_topic").value)
        self._deadband_lin = float(self.get_parameter("deadband_linear").value)
        self._deadband_ang = float(self.get_parameter("deadband_angular").value)

        self._reported_mode = "disarm"

        self._command_pub = self.create_publisher(String, out_topic, 20)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(String, "control/reported_mode", self._mode_callback, 10)

        self.get_logger().info(
            f"H7 Kinematics Node Active. Subscribed to {cmd_vel_topic}, publishing to {out_topic}."
        )

    def _mode_callback(self, msg: String) -> None:
        self._reported_mode = msg.data.strip().lower()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        # Reject motion commands if vehicle is reported DISARM
        if self._reported_mode == "disarm":
            return

        v_lin = msg.linear.x
        w_ang = msg.angular.z

        # Apply deadbands
        if abs(v_lin) < self._deadband_lin:
            v_lin = 0.0
        if abs(w_ang) < self._deadband_ang:
            w_ang = 0.0

        if v_lin == 0.0 and w_ang == 0.0:
            out_msg = String()
            out_msg.data = "stop"
            self._command_pub.publish(out_msg)
            return

        # Differential drive kinematics
        v_left = v_lin - (w_ang * self._wheel_separation / 2.0)
        v_right = v_lin + (w_ang * self._wheel_separation / 2.0)

        # Convert m/s to RPM: rpm = (v * 60) / (2 * pi * r)
        rpm_left = (v_left * 60.0) / (2.0 * math.pi * self._wheel_radius)
        rpm_right = (v_right * 60.0) / (2.0 * math.pi * self._wheel_radius)

        # Scale down if maximum RPM is exceeded while maintaining ratio
        max_requested = max(abs(rpm_left), abs(rpm_right))
        if max_requested > self._max_rpm:
            scale = float(self._max_rpm) / max_requested
            rpm_left *= scale
            rpm_right *= scale

        # Format H7 terminal command
        cmd_str = self._format_h7_command(v_lin, w_ang, rpm_left, rpm_right)
        if cmd_str:
            out_msg = String()
            out_msg.data = cmd_str
            self._command_pub.publish(out_msg)

    def _format_h7_command(
        self, v_lin: float, w_ang: float, rpm_left: float, rpm_right: float
    ) -> str:
        # Direct motion mapping according to H7 terminal parser grammar
        if abs(w_ang) < self._deadband_ang:
            # Straight motion
            rpm_avg = int(round((abs(rpm_left) + abs(rpm_right)) / 2.0))
            rpm_clamped = min(self._max_rpm, max(1, rpm_avg))
            return f"f{rpm_clamped}" if v_lin > 0 else f"b{rpm_clamped}"
        elif abs(v_lin) < self._deadband_lin:
            # Turn in place
            rpm_avg = int(round((abs(rpm_left) + abs(rpm_right)) / 2.0))
            rpm_clamped = min(self._max_rpm, max(1, rpm_avg))
            return f"r{rpm_clamped}" if w_ang < 0 else f"l{rpm_clamped}"
        else:
            # Arc turn motion: drive <rpm|duty> <target> <fl|fr|bl|br> tr <decimal>
            target_rpm = int(round(max(abs(rpm_left), abs(rpm_right))))
            target_rpm = min(self._max_rpm, max(1, target_rpm))
            
            min_rpm = min(abs(rpm_left), abs(rpm_right))
            turn_ratio = min_rpm / float(target_rpm) if target_rpm > 0 else 0.0

            # Determine arc direction
            if v_lin > 0 and w_ang < 0:
                direction = "fr"
            elif v_lin > 0 and w_ang > 0:
                direction = "fl"
            elif v_lin < 0 and w_ang < 0:
                direction = "br"
            else:
                direction = "bl"

            return f"drive rpm {target_rpm} {direction} tr {turn_ratio:.2f}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H7KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
