import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
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
    motor_mag_bridge = Node(
        package='earendil_bot',
        executable='motor_mag_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200
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
        motor_mag_bridge,  # Also reads IMU from Arduino and publishes /imu/data
        
        # GPS (uncomment when plugged in)
        # gps_node,

        # Navigation:
        # Run in a separate terminal:
        # ros2 run earendil_bot gps_nav_test --ros-args -p robot_lat:=... -p base_lat:=...
        # ros2 run earendil_bot imu_turn_test --ros-args -p robot_lat:=... -p base_lat:=...
    ])
