#!/usr/bin/env python3
"""Navigate the rover back to the base (starting) position.

Reads the base position saved by circle_escape.py from
~/.ros/base_position.json and commands Nav2 to navigate there.

Usage:
    ros2 run earendil_bot base_return
"""

import json
import os
import sys
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


BASE_POSITION_FILE = os.path.expanduser('~/.ros/base_position.json')


def load_base_position() -> dict:
    """Load the saved base position from JSON file."""
    if not os.path.exists(BASE_POSITION_FILE):
        raise FileNotFoundError(
            f'Base position file not found: {BASE_POSITION_FILE}\n'
            'Make sure circle_escape was run first to save the base position.'
        )
    with open(BASE_POSITION_FILE, 'r') as f:
        return json.load(f)


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # --- Load saved base position ---
    try:
        base = load_base_position()
    except FileNotFoundError as e:
        navigator.get_logger().error(str(e))
        rclpy.shutdown()
        sys.exit(1)

    navigator.get_logger().info(
        f'📍 Base position loaded: x={base["x"]:.2f}, y={base["y"]:.2f}'
    )

    # --- Wait for Nav2 ---
    navigator.get_logger().info('Waiting for Nav2 bt_navigator to become active …')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Nav2 is active!')
    time.sleep(2.0)

    # --- Build goal pose ---
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = base['x']
    goal_pose.pose.position.y = base['y']
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = base.get('qx', 0.0)
    goal_pose.pose.orientation.y = base.get('qy', 0.0)
    goal_pose.pose.orientation.z = base.get('qz', 0.0)
    goal_pose.pose.orientation.w = base.get('qw', 1.0)

    navigator.get_logger().info(
        f'=== Navigating back to base: '
        f'Map(x={base["x"]:.2f}, y={base["y"]:.2f}) ==='
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
        navigator.get_logger().info('✅ Returned to base!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('⚠️ Navigation was canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('❌ Navigation failed!')
    else:
        navigator.get_logger().error(f'Unknown result: {result}')

    navigator.get_logger().info('base_return shutting down.')
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
