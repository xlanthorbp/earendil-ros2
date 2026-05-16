#!/usr/bin/env python3
"""Drive the rover out of a circular enclosed starting area.

The rover spawns inside a ~3-4 m² circular pen with an opening in front.
This node assumes the rover is **already facing the exit** at start-up.

Strategy
--------
1. Wait for the first LiDAR scan to arrive.
2. Verify the front is clear (opening detected) — if not, rotate slowly
   until it is.
3. Drive forward at a constant speed.
4. Monitor the left and right LiDAR sectors.  While inside the circle
   the side ranges are short (wall nearby).  Once both sides read far
   enough the rover has cleared the enclosure.
5. Continue forward a short clearance distance, then stop.
"""

from __future__ import annotations

import json
import math
import os
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def _sector_mean(ranges: list[float], r_min: float, r_max: float,
                 start_idx: int, end_idx: int) -> float:
    """Return the mean of valid range values in the index window."""
    total = count = 0.0
    for i in range(start_idx, end_idx):
        r = ranges[i % len(ranges)]
        if r_min <= r <= r_max:
            total += r
            count += 1.0
    return (total / count) if count > 0 else r_max


class BaseExit(Node):
    """One-shot node: drive out of the starting enclosure and shut down."""

    def __init__(self) -> None:
        super().__init__('base_exit')

        # ---- Parameters ----
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_escape')

        # Driving speeds
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('angular_speed', 0.4)

        # A side sector reading above this threshold means "open" (no wall)
        self.declare_parameter('side_open_threshold', 3.0)

        # The front sector must have at least this much free space to proceed
        self.declare_parameter('front_clear_threshold', 1.5)

        # After detecting we're outside, drive this many extra meters
        self.declare_parameter('clearance_distance', 1.5)

        # Sector angular widths (degrees) measured from center-front
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_angle_min_deg', 60.0)
        self.declare_parameter('side_angle_max_deg', 120.0)

        # Consecutive scan confirmations before declaring "escaped"
        self.declare_parameter('escape_confirm_count', 5)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.side_open_threshold = float(self.get_parameter('side_open_threshold').value)
        self.front_clear_threshold = float(self.get_parameter('front_clear_threshold').value)
        self.clearance_distance = float(self.get_parameter('clearance_distance').value)
        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.side_angle_min = math.radians(float(self.get_parameter('side_angle_min_deg').value))
        self.side_angle_max = math.radians(float(self.get_parameter('side_angle_max_deg').value))
        self.escape_confirm_needed = int(self.get_parameter('escape_confirm_count').value)

        # ---- State ----
        self.state: str = 'WAIT_SCAN'  # WAIT_SCAN → ALIGN → DRIVE → CLEARANCE → DONE
        self.base_saved: bool = False
        self.escape_confirm: int = 0
        self.clearance_remaining: float = self.clearance_distance
        self.last_stamp: Optional[float] = None

        # ---- TF (for saving base position) ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10)

        # Save starting position for base_return.py using a timer instead of blocking
        self.save_base_timer = self.create_timer(0.5, self._save_base_position)

        self.get_logger().info(
            f"BaseExit ready — subscribing to '{scan_topic}', "
            f"publishing to '{cmd_vel_topic}'."
        )

    # ------------------------------------------------------------------ #
    #  Save base position
    # ------------------------------------------------------------------ #
    def _save_base_position(self) -> None:
        """Look up robot pose in map frame and save to JSON file."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time()
            )
            pos = t.transform.translation
            rot = t.transform.rotation
            base_data = {
                'x': pos.x,
                'y': pos.y,
                'z': pos.z,
                'qx': rot.x,
                'qy': rot.y,
                'qz': rot.z,
                'qw': rot.w,
            }
            save_path = os.path.expanduser('~/.ros/base_position.json')
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            with open(save_path, 'w') as f:
                json.dump(base_data, f, indent=2)
            self.get_logger().info(
                f'📍 Base position saved: x={pos.x:.2f}, y={pos.y:.2f} '
                f'→ {save_path}')
            self.base_saved = True
            self.save_base_timer.cancel()
        except TransformException:
            self.get_logger().info('Waiting for TF to save base position …', throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ #
    #  Helpers
    # ------------------------------------------------------------------ #
    def _angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        """Map an angle (rad, 0 = front) to a LaserScan index."""
        idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
        return idx % len(scan.ranges)

    def _sector_stats(self, scan: LaserScan, angle_from: float,
                      angle_to: float) -> float:
        """Return mean range in the sector [angle_from, angle_to] (radians)."""
        i0 = self._angle_to_index(scan, angle_from)
        i1 = self._angle_to_index(scan, angle_to)
        if i1 < i0:
            i1 += len(scan.ranges)
        return _sector_mean(scan.ranges, scan.range_min, scan.range_max,
                            i0, i1)

    def _publish(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish(0.0, 0.0)

    # ------------------------------------------------------------------ #
    #  Main callback
    # ------------------------------------------------------------------ #
    def _scan_callback(self, scan: LaserScan) -> None:  # noqa: C901
        now = self.get_clock().now().nanoseconds * 1e-9

        # ---- Compute sector ranges ----
        front_mean = self._sector_stats(scan, -self.front_half_angle,
                                         self.front_half_angle)
        left_mean  = self._sector_stats(scan,  self.side_angle_min,
                                         self.side_angle_max)
        right_mean = self._sector_stats(scan, -self.side_angle_max,
                                         -self.side_angle_min)

        # ---- State machine ----
        if self.state == 'WAIT_SCAN':
            if not self.base_saved:
                return  # Wait for position to be saved first

            self.get_logger().info(
                f'First scan received — front={front_mean:.2f} m, '
                f'left={left_mean:.2f} m, right={right_mean:.2f} m')
            self.state = 'ALIGN'
            self.last_stamp = now

        if self.state == 'ALIGN':
            # Front is already clear → go straight
            if front_mean >= self.front_clear_threshold:
                self.get_logger().info(
                    f'Front clear ({front_mean:.2f} m). Driving forward!')
                self.state = 'DRIVE'
                self.last_stamp = now
            else:
                # Rotate slowly to find the opening
                self._publish(0.0, self.angular_speed)
                return

        if self.state == 'DRIVE':
            # Check if both sides are now open → we left the enclosure
            sides_open = (left_mean >= self.side_open_threshold and
                          right_mean >= self.side_open_threshold)
            if sides_open:
                self.escape_confirm += 1
            else:
                self.escape_confirm = 0

            if self.escape_confirm >= self.escape_confirm_needed:
                self.get_logger().info(
                    '✅ Enclosure cleared! Driving a bit more for clearance …')
                self.state = 'CLEARANCE'
                self.clearance_remaining = self.clearance_distance
                self.last_stamp = now
            else:
                self._publish(self.linear_speed, 0.0)

            self.last_stamp = now

        if self.state == 'CLEARANCE':
            dt = now - self.last_stamp if self.last_stamp else 0.0
            self.clearance_remaining -= self.linear_speed * dt
            self.last_stamp = now

            if self.clearance_remaining <= 0.0:
                self._stop()
                self.get_logger().info(
                    '🏁 Circle escape complete — rover is outside the enclosure.')
                self.state = 'DONE'
            else:
                self._publish(self.linear_speed, 0.0)

        if self.state == 'DONE':
            self._stop()
            raise SystemExit


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = BaseExit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
