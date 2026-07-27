import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_bot')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Start LiDAR (STL27L)
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='STL27L',
            output='screen',
            parameters=[
                hardware_params,
                {'product_name': 'LDLiDAR_STL27L'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 0.0},
                {'angle_crop_max': 0.0}
            ]
        ),
        
        # 2. Base link to laser static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_stl27l',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
        ),
        
        # 3. Hardware Bridge Node (Encoders, IMU, Magnetometer, IR, Motor Driver)
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
            parameters=[hardware_params]
        )
    ])
