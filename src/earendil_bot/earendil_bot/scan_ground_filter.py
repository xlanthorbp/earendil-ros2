#!/usr/bin/env python3
"""
scan_ground_filter  -  Removes ground-plane hits from a 2-D LaserScan.

Problem
-------
On rough terrain (hills, craters) the rover tilts. Because the LiDAR is
rigidly mounted, its scan plane tilts too. Forward-facing rays hit the
ground a few metres ahead and Nav2 sees them as impassable walls.

Solution
--------
Read the robot's roll and pitch from the IMU, smooth that estimate, then
compute where each scan ray would intersect a local ground plane. Only
rays that land within an adaptive tolerance band around that predicted
ground-hit distance are removed, which helps preserve closer real
obstacles.
"""

import math
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan, Imu
import tf2_ros
from tf_transformations import euler_from_quaternion


class ScanGroundFilter(Node):
    """Filter ground returns out of a 2-D LaserScan using IMU orientation."""

    def __init__(self):
        super().__init__('scan_ground_filter')

        # ── tuneable parameters ─────────────────────────────────────────
        self.declare_parameter('lidar_height', 0.42)        # fallback sensor height above ground
        # Max allowed deviation from the expected ground-hit distance.
        self.declare_parameter('ground_tolerance', 0.30)
        self.declare_parameter('ground_tolerance_min', 0.05)
        self.declare_parameter('ground_tolerance_scale', 0.08)
        self.declare_parameter('min_pitch_deg', 3.0)        # engage filtering above this tilt
        self.declare_parameter('tilt_hysteresis_deg', 0.5)
        self.declare_parameter('imu_alpha', 0.20)
        self.declare_parameter('scan_topic_in', '/scan')
        self.declare_parameter('scan_topic_out', '/scan_filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('base_frame', 'body_link')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('use_tf_mount_compensation', True)
        self.declare_parameter('use_tf_lidar_height', False)
        self.declare_parameter('tf_lookup_period_sec', 2.0)

        self.configured_lidar_height = float(self.get_parameter('lidar_height').value)
        self.lidar_height = self.configured_lidar_height
        self.ground_tol_max = max(0.0, float(self.get_parameter('ground_tolerance').value))
        self.ground_tol_min = min(
            self.ground_tol_max,
            max(0.0, float(self.get_parameter('ground_tolerance_min').value)),
        )
        self.ground_tol_scale = max(0.0, float(self.get_parameter('ground_tolerance_scale').value))
        self.min_pitch_rad = math.radians(float(self.get_parameter('min_pitch_deg').value))
        self.tilt_hysteresis_rad = math.radians(
            max(0.0, float(self.get_parameter('tilt_hysteresis_deg').value))
        )
        self.imu_alpha = min(1.0, max(0.0, float(self.get_parameter('imu_alpha').value)))
        scan_in = self.get_parameter('scan_topic_in').value
        scan_out = self.get_parameter('scan_topic_out').value
        imu_topic = self.get_parameter('imu_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.use_tf_mount_comp = self.get_parameter('use_tf_mount_compensation').value
        self.use_tf_lidar_height = self.get_parameter('use_tf_lidar_height').value
        self.tf_lookup_period = max(0.1, float(self.get_parameter('tf_lookup_period_sec').value))

        # ── state ───────────────────────────────────────────────────────
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self._imu_initialized = False
        self._filter_active = False
        self._mount_roll = 0.0
        self._mount_pitch = 0.0
        self._last_tf_lookup_time = 0.0
        self._tf_lookup_warned = False

        if self.use_tf_mount_comp or self.use_tf_lidar_height:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # ── QoS ─────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── subscribers ─────────────────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_cb, sensor_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_in, self._scan_cb, sensor_qos)

        # ── publisher ───────────────────────────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, scan_out, sensor_qos)

        self.get_logger().info(
            f'ScanGroundFilter active  h={self.lidar_height:.2f}m  '
            f'tol=[{self.ground_tol_min:.2f}, {self.ground_tol_max:.2f}]m  '
            f'scale={self.ground_tol_scale:.2f}  '
            f'min_pitch={math.degrees(self.min_pitch_rad):.1f}deg  '
            f'hysteresis={math.degrees(self.tilt_hysteresis_rad):.1f}deg  '
            f'imu_alpha={self.imu_alpha:.2f}  '
            f'tf_mount={self.use_tf_mount_comp}  '
            f'tf_height={self.use_tf_lidar_height}'
        )

    # ─────────────────────────────────────────────────────────────────────
    @staticmethod
    def _ema_angle(previous: float, current: float, alpha: float) -> float:
        """Smooth an angle while respecting wraparound."""
        delta = math.atan2(math.sin(current - previous), math.cos(current - previous))
        return previous + alpha * delta

    def _refresh_sensor_pose_from_tf(self):
        """Update LiDAR mount orientation and optional height from TF."""
        if self.tf_buffer is None:
            return

        now = time.monotonic()
        if (now - self._last_tf_lookup_time) < self.tf_lookup_period:
            return
        self._last_tf_lookup_time = now

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            if self.use_tf_mount_comp:
                q = transform.transform.rotation
                mount_roll, mount_pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self._mount_roll = mount_roll
                self._mount_pitch = mount_pitch

            if self.use_tf_lidar_height:
                tf_height = abs(transform.transform.translation.z)
                if tf_height > 1e-3:
                    self.lidar_height = tf_height
                else:
                    self.lidar_height = self.configured_lidar_height

            self._tf_lookup_warned = False
        except Exception:
            if self.use_tf_lidar_height:
                self.lidar_height = self.configured_lidar_height
            if not self._tf_lookup_warned:
                self.get_logger().warn(
                    f'Could not resolve TF {self.base_frame} -> {self.laser_frame}; '
                    'keeping configured LiDAR height and last known mount offsets.'
                )
                self._tf_lookup_warned = True

    def _should_filter_scan(self, pitch: float, roll: float) -> bool:
        """Apply hysteresis so the filter does not chatter near the tilt threshold."""
        upper = self.min_pitch_rad + self.tilt_hysteresis_rad
        lower = max(0.0, self.min_pitch_rad - self.tilt_hysteresis_rad)
        max_tilt = max(abs(pitch), abs(roll))

        if self._filter_active:
            if max_tilt <= lower:
                self._filter_active = False
        elif max_tilt >= upper:
            self._filter_active = True

        return self._filter_active

    def _compute_adaptive_tolerance(self, expected_ground_ranges, downward_mask):
        """Grow tolerance with distance while keeping a tight near-field band."""
        tolerance = np.full(expected_ground_ranges.shape, self.ground_tol_max, dtype=np.float32)
        if np.any(downward_mask):
            tolerance[downward_mask] = np.minimum(
                self.ground_tol_max,
                self.ground_tol_min
                + self.ground_tol_scale * expected_ground_ranges[downward_mask],
            )
        return tolerance

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self._imu_initialized:
            self.current_roll = roll
            self.current_pitch = pitch
            self._imu_initialized = True
            return

        self.current_roll = self._ema_angle(self.current_roll, roll, self.imu_alpha)
        self.current_pitch = self._ema_angle(self.current_pitch, pitch, self.imu_alpha)

    # ─────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self._refresh_sensor_pose_from_tf()

        pitch = self.current_pitch + self._mount_pitch
        roll = self.current_roll + self._mount_roll

        # If the robot is practically flat, just forward the scan unchanged.
        if not self._should_filter_scan(pitch, roll):
            self.scan_pub.publish(msg)
            return

        h = self.lidar_height
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)

        # Build array of scan angles (each ray's horizontal angle in sensor frame)
        angles = np.arange(n, dtype=np.float32) * msg.angle_increment + msg.angle_min

        # ── Geometry ────────────────────────────────────────────────────
        # The LiDAR emits rays in its own XY plane (z=0 in sensor frame).
        # When the body (and therefore the sensor) pitches by θ_p and rolls
        # by θ_r, a ray at scan angle α has a world-frame direction:
        #
        #   d_x =  cos(α) cos(θ_p)
        #   d_y =  sin(α) cos(θ_r) - cos(α) sin(θ_p) sin(θ_r)
        #   d_z = -cos(α) sin(θ_p) cos(θ_r) - sin(α) sin(θ_r)
        #
        # The ray hits the ground (z = -h) at range
        #   r_ground = h / [cos(α) sin(θ_p) cos(θ_r) + sin(α) sin(θ_r)]
        #
        # (only valid when the denominator > 0, i.e. ray points downward)
        # ────────────────────────────────────────────────────────────────

        cos_a = np.cos(angles)
        sin_a = np.sin(angles)
        sp = math.sin(pitch)
        sr = math.sin(roll)
        cr = math.cos(roll)

        # Vertical component of each ray direction (positive = upward in world)
        dz = -(cos_a * sp * cr) - (sin_a * sr)

        # dz < 0 means the ray points into the ground
        ground_mask = dz < -1e-4   # ray has significant downward component

        # Expected ground-hit range for rays that point downward
        r_ground = np.full(n, np.inf, dtype=np.float32)
        r_ground[ground_mask] = h / (-dz[ground_mask])

        # Use a tighter near-field band so closer real obstacles are not
        # removed simply because the expected ground hit is farther away.
        adaptive_tol = self._compute_adaptive_tolerance(r_ground, ground_mask)

        # Determine which rays are hitting the ground.
        valid = np.isfinite(ranges) & (ranges > msg.range_min)
        with np.errstate(invalid='ignore'):
            is_ground = (
                ground_mask
                & valid
                & (np.abs(ranges - r_ground) <= adaptive_tol)
            )

        # Replace ground hits with inf (costmap will ignore them)
        ranges[is_ground] = float('inf')

        # Publish the filtered scan
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges.tolist()
        out.intensities = list(msg.intensities) if msg.intensities else []
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanGroundFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
