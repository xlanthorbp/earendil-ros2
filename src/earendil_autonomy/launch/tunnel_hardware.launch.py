import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_autonomy')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Start Base Hardware (LiDAR, Static TF, H7 Hardware Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(earendil_share_dir, 'launch', 'h7_hardware.launch.py')
            )
        ),

        # 2. Start ArUco Ethernet Receiver Node
        Node(
            package='earendil_autonomy',
            executable='aruco_receiver',
            name='aruco_receiver',
            output='screen',
            parameters=[hardware_params]
        )
    ])
