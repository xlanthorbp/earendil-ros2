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

    # Parameters
    twist_mux_params = os.path.join(earendil_share_dir, 'config', 'twist_mux.yaml')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    return LaunchDescription([
        # 1. Start Lidar (Directly defined so it can read hardware_params.yaml)
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
        
        # Base link to laser transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_stl27l',
            arguments=['0','0','0.18','0','0','0','base_link','base_laser']
        ),
        
        # (IR Bridge has been merged into hardware_bridge)
        
        # 3. Motor Control (Hardware Bridge)
        # Assuming hardware_bridge is your main motor controller from Arduino
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
            parameters=[hardware_params]
        ),

        # 4. Twist Mux (Optional, cmd_vel router)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[('/cmd_vel_out', '/cmd_vel')],
            parameters=[twist_mux_params, {'use_sim_time': False}]
        ),

        # 5. Aruco Ethernet Receiver Node
        Node(
            package='earendil_bot',
            executable='aruco_receiver',
            name='aruco_receiver',
            output='screen',
            parameters=[hardware_params]
        )
    ])
