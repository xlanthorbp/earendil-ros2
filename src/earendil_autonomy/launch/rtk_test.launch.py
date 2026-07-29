import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_autonomy')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    target_lat_launch_conf = LaunchConfiguration('target_lat')
    target_lon_launch_conf = LaunchConfiguration('target_lon')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_lat',
            default_value='0.0',
            description='Target latitude for the navigation test'
        ),
        DeclareLaunchArgument(
            'target_lon',
            default_value='0.0',
            description='Target longitude for the navigation test'
        ),

        # Rover RTK GPS Node
        Node(
            package='earendil_autonomy',
            executable='roverRTK',
            name='roverRTK',
            output='screen',
            parameters=[
                hardware_params,
                {
                    'target_lat': target_lat_launch_conf,
                    'target_lon': target_lon_launch_conf,
                    'enable_test_flow': True
                }
            ]
        ),
    ])
