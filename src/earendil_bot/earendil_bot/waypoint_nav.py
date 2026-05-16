#!/usr/bin/env python3
"""
Autonomous waypoint navigation node for bumpy terrain.

Uses Nav2's BasicNavigator (nav2_simple_commander) to drive the robot
through a sequence of user-defined waypoints.  Designed for rough /
uneven surfaces where progress is slower and tolerances must be relaxed.

Waypoints are loaded from config/missions.yaml.

Usage:
    ros2 run earendil_bot waypoint_nav              # run once
    ros2 run earendil_bot waypoint_nav --ros-args -p loop:=true   # patrol continuously
"""

import os
import math
import time

import yaml

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from ament_index_python.packages import get_package_share_directory


def load_waypoints() -> list:
    """Load waypoints from config/missions.yaml as (lat, lon, yaw_deg) tuples."""
    config_path = os.path.join(
        get_package_share_directory('earendil_bot'),
        'config', 'missions.yaml',
    )
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    missions = data.get('missions', {})
    waypoints = []
    for name, info in missions.items():
        # Default yaw = 0 — missions.yaml doesn't specify yaw
        waypoints.append((info['lat'], info['lon'], 0))
    return waypoints


def create_pose(
    navigator: BasicNavigator,
    node: Node,
    lat: float,
    lon: float,
    yaw_deg: float,
    fromll_client,
) -> PoseStamped:
    """Uses /fromLL service to convert GPS to map frame PoseStamped."""
    while not fromll_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('/fromLL service not available, waiting...')

    req = FromLL.Request()
    req.ll_point.latitude = lat
    req.ll_point.longitude = lon
    req.ll_point.altitude = 0.0

    future = fromll_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = res.map_point.x
    pose.pose.position.y = res.map_point.y
    pose.pose.position.z = 0.0

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()

    # Create a helper node to run the service client
    service_node = rclpy.create_node('waypoint_nav_services')
    navigator = BasicNavigator()

    # --- Wait for Nav2 to be fully active ---
    # NOTE: We use _waitForNodeToActivate directly because
    # waitUntilNav2Active() also waits for AMCL, which this project
    # doesn't use (robot_localization provides map→odom TF instead).
    navigator.get_logger().info('Waiting for Nav2 bt_navigator to become active ...')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Nav2 is active! Giving EKF a few seconds to settle ...')
    time.sleep(5.0)

    # --- Declare parameters ---
    navigator.declare_parameter('loop', False)
    loop_mode = navigator.get_parameter('loop').get_parameter_value().bool_value

    # --- Load waypoints from YAML ---
    waypoints = load_waypoints()

    if not waypoints:
        navigator.get_logger().error(
            'No waypoints found! Check config/missions.yaml.'
        )
        rclpy.shutdown()
        return

    # --- Create /fromLL client once and reuse ---
    fromll_client = service_node.create_client(FromLL, '/fromLL')

    # --- Build waypoint poses ---
    waypoint_poses = []
    for idx, (lat, lon, yaw) in enumerate(waypoints):
        pose = create_pose(navigator, service_node, lat, lon, yaw, fromll_client)
        waypoint_poses.append(pose)
        navigator.get_logger().info(
            f'  Waypoint {idx}: Lat={lat:.6f} Lon={lon:.6f} -> '
            f'Map(x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f})'
        )

    run_count = 0

    while rclpy.ok():
        run_count += 1
        navigator.get_logger().info(
            f'===  Starting waypoint mission (run #{run_count}, '
            f'{len(waypoint_poses)} waypoints)  ==='
        )

        # --- Send all waypoints via NavigateThroughPoses ---
        navigator.goThroughPoses(waypoint_poses)

        # --- Monitor until mission finishes ---
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                current_pose = feedback.current_pose
                nav_time = feedback.navigation_time
                navigator.get_logger().info(
                    f'  Navigating … '
                    f'robot @ ({current_pose.pose.position.x:.2f}, '
                    f'{current_pose.pose.position.y:.2f})  '
                    f'elapsed: {nav_time.sec}s',
                    throttle_duration_sec=5.0,
                )
            time.sleep(0.5)

        # --- Evaluate result ---
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('✅  All waypoints reached successfully!')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('⚠️  Mission was canceled.')
            break
        elif result == TaskResult.FAILED:
            navigator.get_logger().error('❌  Mission failed!')
            break
        else:
            navigator.get_logger().error(f'Unknown result: {result}')
            break

        if not loop_mode:
            break

        navigator.get_logger().info('Loop mode: restarting waypoint sequence in 3 seconds …')
        time.sleep(3.0)

    navigator.get_logger().info('Waypoint navigation node shutting down.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
