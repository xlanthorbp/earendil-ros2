import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Earendil bot package path
    earendil_share_dir = get_package_share_directory('earendil_bot')
    
    # Parameters path
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    # Launch Configurations
    target_lat_launch_conf = LaunchConfiguration('target_lat')
    target_lon_launch_conf = LaunchConfiguration('target_lon')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'target_lat',
            default_value='0.0',
            description='Target latitude for the navigation test (0.0 to prompt in terminal)'
        ),
        DeclareLaunchArgument(
            'target_lon',
            default_value='0.0',
            description='Target longitude for the navigation test (0.0 to prompt in terminal)'
        ),

        # 1. Start H7 Bridge / Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('earendil_h7_bridge'), 'launch', 'h7_sensors.launch.py')
            )
        ),

        # 2. Start Rover RTK GPS Node (test mode)
        Node(
            package='earendil_bot',
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
