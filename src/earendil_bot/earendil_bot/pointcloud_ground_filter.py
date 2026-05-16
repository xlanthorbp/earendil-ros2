#!/usr/bin/env python3
"""
pointcloud_ground_filter  -  Removes ground points from a depth PointCloud2.

The depth camera is useful for close-range obstacle detection, but on sloped
terrain it also sees a dense carpet of ground points that can overwhelm the
local costmap. This node uses the IMU-estimated gravity direction together
with the camera height above ground to remove points that lie on (or below)
the expected local ground plane.

Incoming cloud:
  /camera/points

Outgoing cloud:
  /camera/points_filtered
"""

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf_transformations import quaternion_matrix


class PointCloudGroundFilter(Node):
    """Filter out ground-dominated depth points before they reach Nav2."""

    def __init__(self):
        super().__init__('pointcloud_ground_filter')

        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/camera/points_filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('base_frame', 'body_link')
        self.declare_parameter('sensor_height', 0.412)
        self.declare_parameter('min_obstacle_height', 0.05)
        self.declare_parameter('max_obstacle_height', 1.8)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('point_stride', 6)
        self.declare_parameter('imu_alpha', 0.20)
        self.declare_parameter('tf_timeout_sec', 0.1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_height = float(self.get_parameter('sensor_height').value)
        self.min_obstacle_height = float(self.get_parameter('min_obstacle_height').value)
        self.max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.imu_alpha = min(1.0, max(0.0, float(self.get_parameter('imu_alpha').value)))
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))

        self._up_vector_base = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self._imu_initialized = False
        self._tf_warned = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.imu_sub = self.create_subscription(Imu, imu_topic, self._imu_cb, sensor_qos)
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, sensor_qos
        )
        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)

        self.get_logger().info(
            f'PointCloudGroundFilter active  in={input_topic}  out={output_topic}  '
            f'base={self.base_frame}  h={self.sensor_height:.3f}m  '
            f'obstacle_height=[{self.min_obstacle_height:.2f}, {self.max_obstacle_height:.2f}]m  '
            f'range=[{self.min_range:.2f}, {self.max_range:.2f}]m  '
            f'stride={self.point_stride}'
        )

    @staticmethod
    def _normalize(vec: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vec)
        if norm < 1e-6:
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)
        return (vec / norm).astype(np.float32)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        rot_base_to_world = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        up_world = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        up_base = rot_base_to_world.T @ up_world
        up_base = self._normalize(up_base)

        if not self._imu_initialized:
            self._up_vector_base = up_base
            self._imu_initialized = True
            return

        blended = ((1.0 - self.imu_alpha) * self._up_vector_base) + (self.imu_alpha * up_base)
        self._up_vector_base = self._normalize(blended)

    def _cloud_cb(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=self.tf_timeout,
            )
            self._tf_warned = False
        except Exception:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'Could not resolve TF {self.base_frame} <- {msg.header.frame_id}; '
                    'depth cloud will be skipped until TF becomes available.'
                )
                self._tf_warned = True
            return

        raw_points = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True
        )

        if raw_points.size == 0:
            empty = point_cloud2.create_cloud_xyz32(msg.header, [])
            self.cloud_pub.publish(empty)
            return

        sensor_points = np.column_stack(
            (raw_points['x'], raw_points['y'], raw_points['z'])
        ).astype(np.float32)

        if self.point_stride > 1:
            sensor_points = sensor_points[::self.point_stride]

        rot_sensor_to_base = quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])[:3, :3].astype(np.float32)
        trans_sensor_to_base = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )

        base_points = (sensor_points @ rot_sensor_to_base.T) + trans_sensor_to_base

        ranges = np.linalg.norm(sensor_points, axis=1)
        up_base = self._up_vector_base
        ground_point = trans_sensor_to_base - (self.sensor_height * up_base)
        signed_height = (base_points - ground_point) @ up_base

        keep_mask = (
            np.isfinite(ranges)
            & (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (signed_height >= self.min_obstacle_height)
            & (signed_height <= self.max_obstacle_height)
        )

        filtered_points = sensor_points[keep_mask]
        filtered_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_points.tolist())
        self.cloud_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGroundFilter()
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
