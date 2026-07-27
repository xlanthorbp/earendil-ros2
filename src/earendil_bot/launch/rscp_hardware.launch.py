import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_bot')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Start H7 Hardware, LiDAR, and ArUco Receiver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(earendil_share_dir, 'launch', 'h7_hardware.launch.py')
            )
        ),
        
        # 2. Start RSCP Serial & Protobuf Bridge Node
        Node(
            package='earendil_bot',
            executable='rscp_bridge_node',
            name='rscp_bridge_node',
            output='screen',
            parameters=[hardware_params]
        ),

        # 3. Start Master Mission Manager Node
        Node(
            package='earendil_bot',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[hardware_params]
        ),

        # 4. Start Rock Receiver Node (Jetson Nano Basalt Rock Perception Receiver)
        Node(
            package='earendil_bot',
            executable='rock_receiver',
            name='rock_receiver',
            output='screen',
            parameters=[hardware_params]
        )
    ])
