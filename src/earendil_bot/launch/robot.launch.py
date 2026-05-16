import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_earendil = get_package_share_directory('earendil_bot')

    # ==========================================
    # Configuration Files
    # ==========================================
    twist_mux_params = os.path.join(pkg_earendil, 'config', 'twist_mux.yaml')
    ukf_local_params = os.path.join(pkg_earendil, 'config', 'ukf_local.yaml')
    ukf_global_params = os.path.join(pkg_earendil, 'config', 'ukf_global.yaml')
    navsat_params = os.path.join(pkg_earendil, 'config', 'navsat.yaml')
    nav2_params = os.path.join(pkg_earendil, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_earendil, 'description', 'robot.urdf.xacro')

    # ==========================================
    # Core Robot Nodes
    # ==========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': False
        }]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        parameters=[twist_mux_params, {'use_sim_time': False}]
    )

    # ==========================================
    # Motor Control (Open-Loop Bridge)
    # ==========================================
    simple_motor_bridge = Node(
        package='earendil_bot',
        executable='simple_motor_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
            'wheel_base': 0.6
        }]
    )

    # ==========================================
    # Hardware Driver Nodes (Examples)
    # ==========================================
    # NOTE: Uncomment and adjust these based on your exact hardware

    # GPS (NMEA)
    # gps_node = Node(
    #     package='nmea_navsat_driver',
    #     executable='nmea_serial_driver',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/ttyACM1',
    #         'baud': 9600,
    #         'frame_id': 'gps_link'
    #     }],
    #     remappings=[('fix', '/gps/raw_fix')]
    # )

    return LaunchDescription([
        # Hardware & Core
        robot_state_publisher,
        twist_mux,
        simple_motor_bridge,
        
        # Hardware Driver Nodes (UNCOMMENT WHEN PLUGGED IN)
        # gps_node, # Uncomment this AND the definition above when you plug in GPS!

        # Pure GPS Navigation Script
        # NOTE: Run `ros2 run earendil_bot pure_gps_nav` in a separate terminal 
        # when you want the robot to start driving to the target!
    ])
