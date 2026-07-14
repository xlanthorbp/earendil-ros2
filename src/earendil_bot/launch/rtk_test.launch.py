import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Earendil bot package path
    earendil_share_dir = get_package_share_directory('earendil_bot')
    
    # Parameters path
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Start Motor Control (Hardware Bridge Node)
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
            parameters=[hardware_params]
        ),

        # 2. Start Rover RTK GPS Node (test mode)
        Node(
            package='earendil_bot',
            executable='roverRTK',
            name='roverRTK',
            output='screen',
            parameters=[hardware_params]
        ),
    ])
