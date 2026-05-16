#!/usr/bin/env python3
"""Interactive GPS waypoint mission navigator.

Replaces the separate mission_antenna, mission_crater and mission_lavatube
nodes with a single unified script.  On startup the user picks a mission
from a YAML config file and the rover navigates there via Nav2 + GPS.

Usage:
    ros2 run earendil_bot mission_nav
"""

import os
import sys
import time

import yaml

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────
#  Load missions from YAML
# ─────────────────────────────────────────────
def load_missions() -> dict:
    """Load the mission waypoints from config/missions.yaml."""
    config_path = os.path.join(
        get_package_share_directory('earendil_bot'),
        'config', 'missions.yaml',
    )
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('missions', {})


# ─────────────────────────────────────────────
#  GPS → Map conversion
# ─────────────────────────────────────────────
def gps_to_map_pose(
    navigator: BasicNavigator,
    service_node: Node,
    lat: float,
    lon: float,
    *,
    _client=None,
) -> PoseStamped:
    """Convert a GPS coordinate to a map-frame PoseStamped via /fromLL."""
    if _client is None:
        _client = service_node.create_client(FromLL, '/fromLL')

    while not _client.wait_for_service(timeout_sec=1.0):
        service_node.get_logger().info('/fromLL service not available, waiting …')

    req = FromLL.Request()
    req.ll_point.latitude = lat
    req.ll_point.longitude = lon
    req.ll_point.altitude = 0.0

    future = _client.call_async(req)
    rclpy.spin_until_future_complete(service_node, future)
    res = future.result()

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = res.map_point.x
    pose.pose.position.y = res.map_point.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


# ─────────────────────────────────────────────
#  Interactive mission picker
# ─────────────────────────────────────────────
def pick_mission(missions: dict) -> tuple:
    """Display a menu and return (name, lat, lon)."""
    keys = list(missions.keys())

    print('\n' + '=' * 50)
    print('  🎯  MISSION SELECTOR')
    print('=' * 50)
    for idx, key in enumerate(keys, 1):
        m = missions[key]
        print(f'  {idx}. {m["description"]}  ({key})')
        print(f'     GPS: {m["lat"]:.6f}, {m["lon"]:.6f}')
    print('=' * 50)

    while True:
        try:
            choice = input(f'\nSelect mission [1-{len(keys)}]: ').strip()
            index = int(choice) - 1
            if 0 <= index < len(keys):
                name = keys[index]
                m = missions[name]
                return name, m['lat'], m['lon']
            print(f'  ⚠️  Please enter a number between 1 and {len(keys)}.')
        except (ValueError, EOFError):
            print(f'  ⚠️  Please enter a valid number.')


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────
def main():
    rclpy.init()

    # Load missions from YAML
    missions = load_missions()
    if not missions:
        print('❌ No missions found in config/missions.yaml!')
        rclpy.shutdown()
        sys.exit(1)

    # Interactive selection
    mission_name, target_lat, target_lon = pick_mission(missions)
    print(f'\n  ✅ Selected: {mission_name}\n')

    service_node = rclpy.create_node('mission_nav_services')
    navigator = BasicNavigator()

    # --- Wait for Nav2 ---
    navigator.get_logger().info('Waiting for Nav2 bt_navigator to become active …')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Nav2 is active! Giving EKF a few seconds to settle …')
    time.sleep(3.0)

    # --- Convert GPS to map pose ---
    goal_pose = gps_to_map_pose(navigator, service_node, target_lat, target_lon)
    navigator.get_logger().info(
        f'=== Navigating to {mission_name}: '
        f'GPS({target_lat:.6f}, {target_lon:.6f}) → '
        f'Map(x={goal_pose.pose.position.x:.2f}, '
        f'y={goal_pose.pose.position.y:.2f}) ==='
    )

    # --- Go ---
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            current = feedback.current_pose
            elapsed = feedback.navigation_time
            navigator.get_logger().info(
                f'  Navigating … robot @ ({current.pose.position.x:.2f}, '
                f'{current.pose.position.y:.2f})  elapsed: {elapsed.sec}s',
                throttle_duration_sec=5.0,
            )
        time.sleep(0.5)

    # --- Report result ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f'✅ {mission_name} reached!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('⚠️ Navigation was canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('❌ Navigation failed!')
    else:
        navigator.get_logger().error(f'Unknown result: {result}')

    navigator.get_logger().info('mission_nav shutting down.')
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
