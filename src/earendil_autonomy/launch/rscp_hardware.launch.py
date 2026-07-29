import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    earendil_share_dir = get_package_share_directory('earendil_autonomy')
    hardware_params = os.path.join(earendil_share_dir, 'config', 'hardware_params.yaml')

    rscp_port = LaunchConfiguration('rscp_port')

    return LaunchDescription([
        DeclareLaunchArgument('rscp_port', default_value='/dev/ttyUSB2', description='RSCP client module serial port'),

        # 1. Start Base Hardware (LiDAR, Static TF, H7 Hardware Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(earendil_share_dir, 'launch', 'h7_hardware.launch.py')
            )
        ),
        
        # 2. Start RSCP Serial & Protobuf Bridge Node
        Node(
            package='earendil_autonomy',
            executable='rscp_bridge_node',
            name='rscp_bridge_node',
            output='screen',
            parameters=[
                hardware_params,
                {'port': rscp_port}
            ]
        ),

        # 3. Start Master Mission Manager Node
        Node(
            package='earendil_autonomy',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[hardware_params]
        ),

        # 4. Start Autonomous GPS Navigator Node (Stage 1-4 Navigation Driver)
        Node(
            package='earendil_autonomy',
            executable='gps_navigator_node',
            name='gps_navigator_node',
            output='screen',
            parameters=[hardware_params]
        ),

        # 5. Start ArUco Ethernet Receiver Node
        Node(
            package='earendil_autonomy',
            executable='aruco_receiver',
            name='aruco_receiver',
            output='screen',
            parameters=[hardware_params]
        ),

        # 6. Start Rock Receiver Node (Basalt Rock Perception Receiver)
        Node(
            package='earendil_autonomy',
            executable='rock_receiver',
            name='rock_receiver',
            output='screen',
            parameters=[hardware_params]
        ),

        # 7. Start RTK GPS Receiver Node (Publishes /gps/fix)
        Node(
            package='earendil_autonomy',
            executable='roverRTK',
            name='roverRTK',
            output='screen',
            parameters=[hardware_params]
        ),

        # 8. Start Peak Finder Node (Stage 1 Summit Search)
        Node(
            package='earendil_autonomy',
            executable='peak_finder',
            name='peak_finder',
            output='screen',
            parameters=[hardware_params]
        ),

        # 9. Start Tunnel Test 5 Node (Stage 3 Lava Tube Exploration)
        Node(
            package='earendil_autonomy',
            executable='tunnel_test5',
            name='tunnel_test5',
            output='screen',
            parameters=[hardware_params]
        ),

        # 10. Start Base Enter Node (Stage 4 Airlock Docking)
        Node(
            package='earendil_autonomy',
            executable='base_enter',
            name='base_enter',
            output='screen',
            parameters=[hardware_params]
        )
    ])
