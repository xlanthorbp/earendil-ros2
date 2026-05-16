#!/usr/bin/env python3
"""Drive through the tunnel using ArUco gate markers.

Uses OpenCV ArUco detection internally — no aruco_ros dependency.

State machine
─────────────
  SEARCH_ENTRY  → Rotate in place until both markers (ID 100 & 101) are visible.
  ALIGN_ENTRY   → Drive toward the midpoint between the entry markers.
                   Tracks the target in map frame so it survives markers
                   leaving the camera FOV as the robot gets close.
                   Transitions once the robot reaches the midpoint.
  IN_TUNNEL     → Drive straight forward. After travelling a minimum
                   distance from the entry gate, start watching for
                   both markers to reappear (exit gate).
  ALIGN_EXIT    → Drive toward the midpoint between the exit markers.
                   When the robot reaches the midpoint, stop and exit.

Usage:
    ros2 run earendil_bot tunnel_aruco_nav
"""

from __future__ import annotations

import math
import sys
from typing import Dict, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa – registers PoseStamped transform


# ──────────────────────────────────────────────────────────────────────
#  OpenCV dictionary map
# ──────────────────────────────────────────────────────────────────────
DICT_MAP = {
    'DICT_4X4_50': aruco.DICT_4X4_50,
    'DICT_4X4_100': aruco.DICT_4X4_100,
    'DICT_4X4_250': aruco.DICT_4X4_250,
    'DICT_5X5_50': aruco.DICT_5X5_50,
    'DICT_5X5_100': aruco.DICT_5X5_100,
    'DICT_5X5_250': aruco.DICT_5X5_250,
    'DICT_6X6_50': aruco.DICT_6X6_50,
    'DICT_6X6_250': aruco.DICT_6X6_250,
    'DICT_7X7_50': aruco.DICT_7X7_50,
    'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
}


# ──────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


# ──────────────────────────────────────────────────────────────────────
#  Node
# ──────────────────────────────────────────────────────────────────────
class TunnelArucoNavigator(Node):

    # States
    SEARCH_ENTRY = 'SEARCH_ENTRY'
    ALIGN_ENTRY = 'ALIGN_ENTRY'
    IN_TUNNEL = 'IN_TUNNEL'
    ALIGN_EXIT = 'ALIGN_EXIT'
    FINISHED = 'FINISHED'

    def __init__(self) -> None:
        super().__init__('tunnel_aruco_nav')

        # ── Parameters ────────────────────────────────────────────────
        # Vision
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size', 1.0)
        self.declare_parameter('dictionary', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('camera_frame', 'camera_link_optical')

        # Navigation
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_tunnel')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('allowed_ids', [100, 101])

        # Speeds
        self.declare_parameter('search_angular_speed', 0.4)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('tunnel_drive_speed', 0.6)

        # Thresholds
        self.declare_parameter('midpoint_reach_threshold', 0.5)
        self.declare_parameter('min_tunnel_travel', 3.0)
        self.declare_parameter('marker_timeout_sec', 1.0)

        # ── Read parameters ───────────────────────────────────────────
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        dict_name = str(self.get_parameter('dictionary').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.allowed_ids = set(
            int(i) for i in self.get_parameter('allowed_ids').value
        )

        self.search_angular_speed = float(
            self.get_parameter('search_angular_speed').value
        )
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.tunnel_drive_speed = float(
            self.get_parameter('tunnel_drive_speed').value
        )

        self.midpoint_reach_threshold = float(
            self.get_parameter('midpoint_reach_threshold').value
        )
        self.min_tunnel_travel = float(
            self.get_parameter('min_tunnel_travel').value
        )
        self.marker_timeout = Duration(
            seconds=float(self.get_parameter('marker_timeout_sec').value)
        )

        # ── OpenCV ArUco setup ────────────────────────────────────────
        if dict_name not in DICT_MAP:
            self.get_logger().error(
                f'Unknown dictionary "{dict_name}". '
                f'Available: {list(DICT_MAP.keys())}')
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')

        self.aruco_dict = aruco.getPredefinedDictionary(DICT_MAP[dict_name])
        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ── ROS I/O ──────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Camera subscriptions
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(
            Image, image_topic, self._image_cb, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State variables ───────────────────────────────────────────
        self.state = self.SEARCH_ENTRY

        # Detected marker positions in base_footprint frame
        # Dict[int, Tuple[float, float]]  →  {id: (x, y)}
        self.detected_markers: Dict[int, Tuple[float, float]] = {}
        self.detected_markers_stamp = self.get_clock().now()

        # Map-frame targets (updated whenever markers are visible)
        self.entry_target_map: Optional[Tuple[float, float]] = None
        self.exit_target_map: Optional[Tuple[float, float]] = None
        self.entry_midpoint_map: Optional[Tuple[float, float]] = None

        # ── Control loop at 10 Hz ────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'TunnelArucoNav started (OpenCV {dict_name}) — '
            f'image: {image_topic}, cmd_vel: {self.cmd_vel_topic}. '
            f'State: {self.state}'
        )

    # ────────────────────────────────────────────────────────────
    #  Camera callbacks
    # ────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        """Cache camera intrinsics from CameraInfo."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg: Image) -> None:
        """Detect ArUco markers and update self.detected_markers."""
        if self.camera_matrix is None:
            return

        # Convert ROS Image → OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            return

        # Estimate pose for each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        new_detections: Dict[int, Tuple[float, float]] = {}

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self.allowed_ids:
                continue

            tvec = tvecs[i][0]

            # Build pose in camera optical frame
            cam_pose = PoseStamped()
            cam_pose.header.stamp = msg.header.stamp
            cam_pose.header.frame_id = self.camera_frame
            cam_pose.pose.position.x = float(tvec[0])
            cam_pose.pose.position.y = float(tvec[1])
            cam_pose.pose.position.z = float(tvec[2])

            # Convert rvec to quaternion
            rvec = rvecs[i][0]
            rot_mat, _ = cv2.Rodrigues(rvec)
            qw, qx, qy, qz = rotation_matrix_to_quat(rot_mat)
            cam_pose.pose.orientation.x = qx
            cam_pose.pose.orientation.y = qy
            cam_pose.pose.orientation.z = qz
            cam_pose.pose.orientation.w = qw

            # Transform to base_footprint
            try:
                base_pose = self.tf_buffer.transform(
                    cam_pose, self.base_frame,
                    timeout=Duration(seconds=0.05))
                bx = base_pose.pose.position.x
                by = base_pose.pose.position.y
            except TransformException:
                # Fallback: rough manual transform (camera optical z→base x)
                bx = float(tvec[2])
                by = float(-tvec[0])

            new_detections[marker_id] = (bx, by)

        if new_detections:
            self.detected_markers = new_detections
            self.detected_markers_stamp = self.get_clock().now()
            self.get_logger().info(
                f'Detected markers: {list(new_detections.keys())}',
                throttle_duration_sec=2.0)

    # ────────────────────────────────────────────────────────────
    #  Gate detection — returns midpoint in base frame
    # ────────────────────────────────────────────────────────────
    def _get_gate_pair(self) -> Optional[Dict]:
        """Return info about the two allowed markers if both are visible.

        All coordinates are in base_footprint frame:
          mid_x  – forward distance to midpoint
          mid_y  – lateral offset of midpoint (+ = left)
          dist   – straight-line distance to midpoint
          heading – angle to midpoint from robot front
          width  – distance between the two markers
        """
        if not self.detected_markers:
            return None
        if (self.get_clock().now() - self.detected_markers_stamp
                > self.marker_timeout):
            return None

        found = {mid: pos for mid, pos in self.detected_markers.items()
                 if mid in self.allowed_ids}

        if len(found) < 2:
            return None

        positions = list(found.values())
        mid_x = (positions[0][0] + positions[1][0]) / 2.0
        mid_y = (positions[0][1] + positions[1][1]) / 2.0
        width = math.hypot(
            positions[1][0] - positions[0][0],
            positions[1][1] - positions[0][1],
        )

        return {
            'mid_x': mid_x,
            'mid_y': mid_y,
            'dist': math.hypot(mid_x, mid_y),
            'heading': math.atan2(mid_y, mid_x),
            'width': width,
        }

    # ────────────────────────────────────────────────────────────
    #  TF helpers
    # ────────────────────────────────────────────────────────────
    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of the robot in the map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None
        tr = t.transform.translation
        rot = t.transform.rotation
        return (tr.x, tr.y, quat_to_yaw(rot.z, rot.w))

    def _base_to_map(
        self, bx: float, by: float, robot_pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """Transform a point from base_footprint to map frame."""
        rx, ry, ryaw = robot_pose
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        return (
            rx + cos_y * bx - sin_y * by,
            ry + sin_y * bx + cos_y * by,
        )

    # ────────────────────────────────────────────────────────────
    #  Motion helpers
    # ────────────────────────────────────────────────────────────
    def _publish_cmd(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _drive_to_map_point(
        self,
        target_map: Tuple[float, float],
        robot_pose: Tuple[float, float, float],
    ) -> bool:
        """P-controller that steers the robot toward a map-frame point.

        Returns True if the robot has reached the target.
        """
        dx = target_map[0] - robot_pose[0]
        dy = target_map[1] - robot_pose[1]
        dist = math.hypot(dx, dy)

        if dist < self.midpoint_reach_threshold:
            self._stop()
            return True

        # Heading error
        desired_yaw = math.atan2(dy, dx)
        heading_err = normalize_angle(desired_yaw - robot_pose[2])

        # Angular command
        angular = clamp(
            self.angular_gain * heading_err,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # Linear speed — reduced when heading is off, zero if way off
        if abs(heading_err) > 0.8:
            linear = 0.0
        else:
            heading_factor = max(0.0, 1.0 - abs(heading_err) / 1.0)
            linear = (
                clamp(dist * 0.5, 0.15, self.max_linear_speed) * heading_factor
            )

        self._publish_cmd(linear, angular)
        return False

    # ────────────────────────────────────────────────────────────
    #  Main control loop
    # ────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        if self.state == self.FINISHED:
            self._stop()
            return

        if self.state == self.SEARCH_ENTRY:
            self._do_search_entry()
        elif self.state == self.ALIGN_ENTRY:
            self._do_align_entry()
        elif self.state == self.IN_TUNNEL:
            self._do_in_tunnel()
        elif self.state == self.ALIGN_EXIT:
            self._do_align_exit()

    # ── State handlers ────────────────────────────────────────────

    def _do_search_entry(self) -> None:
        """Rotate in place until both markers are visible."""
        gate = self._get_gate_pair()
        if gate is not None:
            robot_pose = self._get_robot_pose()
            if robot_pose is not None:
                # Lock the entry target in map frame — never change it again
                self.entry_target_map = self._base_to_map(
                    gate['mid_x'], gate['mid_y'], robot_pose
                )
                self.get_logger().info(
                    f'🔒 Entry gate LOCKED! Distance {gate["dist"]:.2f}m, '
                    f'width {gate["width"]:.2f}m. '
                    f'Map target: ({self.entry_target_map[0]:.2f}, '
                    f'{self.entry_target_map[1]:.2f}). Aligning …'
                )
                self.state = self.ALIGN_ENTRY
                self._stop()
                return

        # Keep rotating to find the markers
        self._publish_cmd(0.0, self.search_angular_speed)

    def _do_align_entry(self) -> None:
        """Drive toward the LOCKED midpoint between entry markers."""
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self._stop()
            return

        # entry_target_map is already locked from SEARCH_ENTRY
        if self.entry_target_map is None:
            self.get_logger().warn('No entry target locked. Going back to search …')
            self.state = self.SEARCH_ENTRY
            self._stop()
            return

        # Log progress periodically
        dx = self.entry_target_map[0] - robot_pose[0]
        dy = self.entry_target_map[1] - robot_pose[1]
        dist_to_target = math.hypot(dx, dy)
        self.get_logger().info(
            f'[ALIGN_ENTRY] Driving to entry midpoint — '
            f'distance remaining: {dist_to_target:.2f}m',
            throttle_duration_sec=3.0
        )

        # Drive toward the locked map-frame target
        reached = self._drive_to_map_point(self.entry_target_map, robot_pose)
        if reached:
            self.entry_midpoint_map = self.entry_target_map
            self.get_logger().info(
                '✅ Positioned between ENTRY markers. '
                'Now driving through tunnel …'
            )
            self.state = self.IN_TUNNEL

    def _do_in_tunnel(self) -> None:
        """Drive straight forward. Watch for exit markers once far enough."""
        robot_pose = self._get_robot_pose()

        # Check if we've travelled far enough from the entry gate
        far_enough = False
        travel_dist = 0.0
        if robot_pose is not None and self.entry_midpoint_map is not None:
            dx = robot_pose[0] - self.entry_midpoint_map[0]
            dy = robot_pose[1] - self.entry_midpoint_map[1]
            travel_dist = math.hypot(dx, dy)
            far_enough = travel_dist >= self.min_tunnel_travel

        self.get_logger().info(
            f'[IN_TUNNEL] Travelled {travel_dist:.2f}m from entry '
            f'(need {self.min_tunnel_travel:.1f}m). '
            f'Far enough: {far_enough}',
            throttle_duration_sec=3.0
        )

        if far_enough:
            gate = self._get_gate_pair()
            if gate is not None and gate['mid_x'] > 0:
                # Lock exit target in map frame
                if robot_pose is not None:
                    self.exit_target_map = self._base_to_map(
                        gate['mid_x'], gate['mid_y'], robot_pose
                    )
                    self.get_logger().info(
                        f'🔒 Exit gate LOCKED at {gate["dist"]:.2f}m! '
                        f'Map target: ({self.exit_target_map[0]:.2f}, '
                        f'{self.exit_target_map[1]:.2f}). Aligning …'
                    )
                    self.state = self.ALIGN_EXIT
                    return

        # Drive straight
        self._publish_cmd(self.tunnel_drive_speed, 0.0)

    def _do_align_exit(self) -> None:
        """Drive toward LOCKED exit gate midpoint. Stop when reached."""
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self._stop()
            return

        # exit_target_map is already locked from IN_TUNNEL
        if self.exit_target_map is None:
            # Haven't locked a target yet — creep forward
            self._publish_cmd(0.3, 0.0)
            return

        # Log progress periodically
        dx = self.exit_target_map[0] - robot_pose[0]
        dy = self.exit_target_map[1] - robot_pose[1]
        dist_to_target = math.hypot(dx, dy)
        self.get_logger().info(
            f'[ALIGN_EXIT] Driving to exit midpoint — '
            f'distance remaining: {dist_to_target:.2f}m',
            throttle_duration_sec=3.0
        )

        # Drive toward the locked map-frame target
        reached = self._drive_to_map_point(self.exit_target_map, robot_pose)
        if reached:
            self._stop()
            self.get_logger().info(
                '✅ Positioned between EXIT markers. Mission complete!'
            )
            self.state = self.FINISHED
            raise SystemExit


# ──────────────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = TunnelArucoNavigator()
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
