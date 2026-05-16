#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration  # Import for Duration class
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import time

def create_pose(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # --- Wait for Nav2 to be fully active ---
    # Note: Do NOT use waitUntilNav2Active() here — it waits for AMCL by default,
    # but this project uses EKF + NavSat (robot_localization) for localization.
    navigator.get_logger().info('Waiting for Nav2 bt_navigator to become active ...')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Nav2 is active!')

    # --- Record Start Pose using TF2 ---
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)

    current_pose = None
    start_x = 0.0
    start_y = 0.0
    start_yaw = 0.0

    navigator.get_logger().info('Looking for robot pose in TF...')
    
    while rclpy.ok() and current_pose is None:
        try:
            # Using a timeout in lookup_transform is more robust for scripts
            t = tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                rclpy.time.Time(), 
                timeout=Duration(seconds=1.0)
            )
            
            start_x = t.transform.translation.x
            start_y = t.transform.translation.y
            
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            start_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            
            current_pose = True  
            break
        except Exception as e:
            navigator.get_logger().warn(f'TF Lookup failed: {str(e)}. Retrying...')
            rclpy.spin_once(navigator, timeout_sec=0.1)
            time.sleep(1.0)

    navigator.get_logger().info(f'Starting mission at ({start_x:.2f}, {start_y:.2f}) with yaw {start_yaw:.2f} rad')

    # --- Step 1: Forward 10 Meters ---
    forward_x = start_x + 10.0 * math.cos(start_yaw)
    forward_y = start_y + 10.0 * math.sin(start_yaw)
    forward_yaw = start_yaw  # Robot ileri giderken yönü değişmez
    
    forward_pose = create_pose(navigator, forward_x, forward_y, forward_yaw)

    navigator.get_logger().info('--- Moving forward 10 meters ---')
    navigator.goToPose(forward_pose)

    while not navigator.isTaskComplete():
        time.sleep(0.5)

    if navigator.getResult() != TaskResult.SUCCEEDED:
        navigator.get_logger().error('Failed to move forward 10 meters!')
        rclpy.shutdown()
        return

    # --- Step 2: Circle around the INITIAL starting point ---
    navigator.get_logger().info('--- Starting circular path around initial start point ---')
    circle_waypoints = []
    radius = 10.0
    num_points = 10  # 36 degree intervals (360 / 10 = 36)
    
    # Center point is the robot's initial starting position
    center_x = start_x
    center_y = start_y
    
    start_angle = forward_yaw
    
    # Calculate waypoints for the circular path
    for i in range(1, num_points + 1):
        angle = start_angle + (2.0 * math.pi / num_points) * i
        
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        # Heading: Look direction tangent to the circle path
        yaw = angle + (math.pi / 2.0)
        
        pose = create_pose(navigator, x, y, yaw)
        circle_waypoints.append(pose)

    navigator.followWaypoints(circle_waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'Executing waypoint {feedback.current_waypoint + 1}/{len(circle_waypoints)}',
                throttle_duration_sec=2.0
            )
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('✅ Circle mission completed successfully!')
    else:
        navigator.get_logger().info('❌ Circle mission failed or canceled.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
