import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_autonomy')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    transport_type = LaunchConfiguration('transport_type')
    serial_device = LaunchConfiguration('serial_device')
    baud_rate = LaunchConfiguration('baud_rate')
    tcp_host = LaunchConfiguration('tcp_host')
    tcp_port = LaunchConfiguration('tcp_port')
    frame_id = LaunchConfiguration('frame_id')
    calibration_file = LaunchConfiguration('calibration_file')
    heading_offset_deg = LaunchConfiguration('heading_offset_deg')
    heading_filter_alpha = LaunchConfiguration('heading_filter_alpha')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('transport_type', default_value='serial', description='Transport connection mode: serial or tcp'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyACM0', description='H7 serial device path'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='H7 serial baud rate'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1', description='TCP bridge host IP'),
        DeclareLaunchArgument('tcp_port', default_value='5000', description='TCP bridge port'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('calibration_file', default_value='~/.config/earendil/mag_calibration.json'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('heading_filter_alpha', default_value='0.20'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='Twist velocity topic'),

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
        
        # 2. Base footprint to laser static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_laser_stl27l',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_footprint', 'base_laser']
        ),

        # 3. STM32H7 Seri / TCP Taşımak Düğümü (h7_serial_node)
        Node(
            package='earendil_autonomy',
            executable='h7_serial_node',
            name='h7_serial_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {
                    'transport_type': transport_type,
                    'serial_device': serial_device,
                    'baud_rate': ParameterValue(baud_rate, value_type=int),
                    'tcp_host': tcp_host,
                    'tcp_port': ParameterValue(tcp_port, value_type=int),
                }
            ]
        ),

        # 4. STM32H7 Komut ve Mod Süpervizörü (h7_command_node)
        Node(
            package='earendil_autonomy',
            executable='h7_command_node',
            name='h7_command_node',
            namespace='earendil',
            output='screen',
            parameters=[hardware_params]
        ),

        # 5. H7 Kinematik Komut Dönüştürücü (h7_kinematics_node)
        Node(
            package='earendil_autonomy',
            executable='h7_kinematics_node',
            name='h7_kinematics_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {
                    'cmd_vel_topic': cmd_vel_topic,
                    'output_command_topic': 'h7/command',
                }
            ]
        ),

        # 6. Pusula / Heading Hesabı Düğümü (heading_node)
        Node(
            package='earendil_autonomy',
            executable='heading_node',
            name='heading_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {
                    'calibration_file': calibration_file,
                    'frame_id': frame_id,
                    'heading_offset_deg': ParameterValue(heading_offset_deg, value_type=float),
                    'filter_alpha': ParameterValue(heading_filter_alpha, value_type=float),
                }
            ]
        ),

        # 7. MPU9250 IMU Düğümü (imu_node)
        Node(
            package='earendil_autonomy',
            executable='imu_node',
            name='imu_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {'frame_id': frame_id}
            ]
        ),

        # 8. QMC5883L Manyetometre Düğümü (magnetometer_node)
        Node(
            package='earendil_autonomy',
            executable='magnetometer_node',
            name='magnetometer_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {'frame_id': frame_id}
            ]
        ),

        # 9. Teker Telemetrisi ve İskelet Durumu Düğümü (wheel_telemetry_node)
        Node(
            package='earendil_autonomy',
            executable='wheel_telemetry_node',
            name='wheel_telemetry_node',
            namespace='earendil',
            output='screen',
            parameters=[
                hardware_params,
                {'frame_id': frame_id}
            ]
        ),
    ])
