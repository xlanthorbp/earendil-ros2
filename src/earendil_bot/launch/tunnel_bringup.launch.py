import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Earendil bot package path
    earendil_share_dir = get_package_share_directory('earendil_bot')
    
    # Lidar package path
    ldlidar_share_dir = get_package_share_directory('ldlidar_stl_ros2')

    # Parameter file path
    tunnel_params_path = os.path.join(earendil_share_dir, 'config', 'tunnel_params.yaml')

    return LaunchDescription([
        # 1. Start Lidar (Calls stl27l.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ldlidar_share_dir, 'launch', 'stl27l.launch.py')
            )
        ),
        
        # 2. Start SHARP Infrared Sensor Bridge
        Node(
            package='earendil_bot',
            executable='ir_bridge',
            name='ir_bridge',
            output='screen'
        ),
        
        # 3. Start Tunnel Navigation Node (With parameters)
        Node(
            package='earendil_bot',
            executable='tunnel_navigator',
            name='tunnel_navigator',
            output='screen',
            parameters=[tunnel_params_path]
        )
    ])
