#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
#
# Sistemdeki tüm donanımların, sensörlerin ve alıcıların (Lidar, IR, Odometri, ArUco, Rock, IMU, Magnetometer, GPS, RSCP Stage)
# veri akış durumunu ve gecikme sürelerini anlık denetleyen donanım sağlık denetim düğümüdür.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Float64, Int32


class HardwareCheckerNode(Node):
    def __init__(self):
        super().__init__('hardware_checker')
        
        # Dictionary to keep track of the last time data was received
        self.last_msg_times = {
            'Lidar (/scan)': 0.0,
            'Infrared (/ir_top)': 0.0,
            'Motor/Encoder (/odom)': 0.0,
            'ArUco Camera (/aruco_visible)': 0.0,
            'Rock Camera (/rock_visible)': 0.0,
            'IMU (/imu/data_raw)': 0.0,
            'Magnetometer (/mag/heading)': 0.0,
            'GPS Fix (/gps/fix)': 0.0
        }

        self.current_stage = 0

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Range, '/ir_top', self.ir_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/aruco_visible', self.aruco_cb, 10)
        self.create_subscription(Bool, '/rock_visible', self.rock_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Int32, '/rscp/command/set_stage', self.stage_cb, 10)

        # 1-second timer to print status to the screen
        self.create_timer(1.0, self.print_status)
        self.get_logger().info('Hardware checker started. Listening for system streams...\n')

    def scan_cb(self, msg):
        self.last_msg_times['Lidar (/scan)'] = self.get_clock().now().nanoseconds / 1e9

    def ir_cb(self, msg):
        self.last_msg_times['Infrared (/ir_top)'] = self.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg):
        self.last_msg_times['Motor/Encoder (/odom)'] = self.get_clock().now().nanoseconds / 1e9

    def aruco_cb(self, msg):
        self.last_msg_times['ArUco Camera (/aruco_visible)'] = self.get_clock().now().nanoseconds / 1e9

    def rock_cb(self, msg):
        self.last_msg_times['Rock Camera (/rock_visible)'] = self.get_clock().now().nanoseconds / 1e9

    def imu_cb(self, msg):
        self.last_msg_times['IMU (/imu/data_raw)'] = self.get_clock().now().nanoseconds / 1e9

    def mag_cb(self, msg):
        self.last_msg_times['Magnetometer (/mag/heading)'] = self.get_clock().now().nanoseconds / 1e9

    def gps_cb(self, msg):
        self.last_msg_times['GPS Fix (/gps/fix)'] = self.get_clock().now().nanoseconds / 1e9

    def stage_cb(self, msg: Int32):
        self.current_stage = msg.data

    def print_status(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Clear terminal screen (for better readability)
        print("\033[H\033[J", end="")
        print("="*55)
        print(" HARDWARE STATUS CHECK ".center(55, "="))
        print("="*55)
        if self.current_stage > 0:
            print(f" CURRENT MISSION STAGE: STAGE {self.current_stage}".center(55))
            print("="*55)

        all_ok = True

        for name, last_time in self.last_msg_times.items():
            if last_time == 0.0:
                print(f"[\033[91mERROR\033[0m] {name:32} -> No data received!")
                all_ok = False
            else:
                elapsed = current_time - last_time
                if elapsed > 1.5:
                    print(f"[\033[93mWARNING\033[0m] {name:32} -> Data stream stopped ({elapsed:.1f}s ago)")
                    all_ok = False
                else:
                    print(f"[\033[92m ACTIVE \033[0m] {name:32} -> Delay: {elapsed:.2f}s")

        print("="*55)
        if all_ok:
            print(" RESULT: All monitored hardware streams are operational! \n")
        else:
            print(" RESULT: Warnings/Errors detected in some hardware streams. \n")


def main(args=None):
    rclpy.init(args=args)
    node = HardwareCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
