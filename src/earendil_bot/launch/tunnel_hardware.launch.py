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

    # Twist Mux Params (if needed for arbitration)
    twist_mux_params = os.path.join(earendil_share_dir, 'config', 'twist_mux.yaml')

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
        
        # 3. Motor Control (Hardware Bridge)
        # Assuming hardware_bridge is your main motor controller from Arduino
        Node(
            package='earendil_bot',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'min_pwm': 65,
                'max_pwm': 90
            }]
        ),

        # 4. Twist Mux (Optional, cmd_vel router)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[('/cmd_vel_out', '/cmd_vel')],
            parameters=[twist_mux_params, {'use_sim_time': False}]
        )
    ])
