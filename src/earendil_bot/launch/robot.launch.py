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
    slam_params = os.path.join(pkg_earendil, 'config', 'mapper_params_online_async.yaml')
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
    # ros2_control Nodes (Motor Control)
    # ==========================================
    controllers_file = os.path.join(pkg_earendil, 'config', 'controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': Command(['xacro ', xacro_file]), 'use_sim_time': False}, controllers_file],
        output="both",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # ==========================================
    # Hardware Driver Nodes (Examples)
    # ==========================================
    # NOTE: Uncomment and adjust these based on your exact hardware

    # LiDAR (RPLIDAR)
    # rplidar_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB1',
    #         'frame_id': 'laser_frame',
    #         'angle_compensate': True,
    #         'scan_mode': 'Standard'
    #     }]
    # )

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

    # IMU (ICM20948 or similar)
    # imu_node = Node(
    #     package='ros2_icm20948', # Or whichever driver you use
    #     executable='icm20948_node',
    #     output='screen',
    #     parameters=[{'frame_id': 'imu_link'}],
    #     remappings=[('imu/data_raw', '/imu/data')]
    # )

    # Depth Camera (RealSense)
    # camera_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    #     )]),
    #     launch_arguments={'enable_pointcloud': 'true', 'align_depth.enable': 'true'}.items()
    # )

    # ==========================================
    # Localization & Sensor Fusion (Dual-UKF)
    # ==========================================
    ukf_local = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_local_node',
        output='screen',
        parameters=[ukf_local_params],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ukf_global = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_global_node',
        output='screen',
        parameters=[ukf_global_params],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/raw_fix'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps'),
            ('odometry/filtered', '/odometry/global')
        ]
    )

    # ==========================================
    # Navigation & Mapping
    # ==========================================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params
        }.items()
    )

    # ==========================================
    # Utility Nodes
    # ==========================================
    scan_ground_filter = Node(
        package='earendil_bot',
        executable='scan_ground_filter',
        output='screen'
    )

    pointcloud_ground_filter = Node(
        package='earendil_bot',
        executable='pointcloud_ground_filter',
        output='screen'
    )

    # ==========================================
    # Launch Sequencing
    # ==========================================
    # Delaying heavy nodes slightly to allow URDF and hardware to initialize
    delay_slam = TimerAction(
        period=3.0,
        actions=[slam_toolbox]
    )

    delay_nav2 = TimerAction(
        period=6.0,
        actions=[nav2_bringup]
    )

    return LaunchDescription([
        # Hardware & Core
        robot_state_publisher,
        twist_mux,
        controller_manager,
        diff_drive_spawner,
        joint_state_spawner,
        # rplidar_node,
        # gps_node,
        # imu_node,
        # camera_node,

        # Localization
        ukf_local,
        ukf_global,
        navsat_transform,

        # Utilities
        scan_ground_filter,
        pointcloud_ground_filter,

        # Mapping & Navigation
        delay_slam,
        delay_nav2
    ])
