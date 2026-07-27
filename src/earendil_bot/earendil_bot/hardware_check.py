#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class HardwareCheckerNode(Node):
    def __init__(self):
        super().__init__('hardware_checker')
        
        # Dictionary to keep track of the last time data was received
        self.last_msg_times = {
            'Lidar (/scan)': 0.0,
            'Infrared (/ir_top)': 0.0,
            'Motor/Encoder (/odom)': 0.0,
            'Camera (/aruco_visible)': 0.0,
            'IMU (/earendil/imu/data_raw)': 0.0,
            'Heading (/earendil/heading/deg)': 0.0,
            'GPS (/gps/fix)': 0.0
        }

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Range, '/ir_top', self.ir_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/aruco_visible', self.cam_cb, 10)
        
        # Added new hardware (IMU, Mag, GPS)
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import Float64
        
        self.create_subscription(Imu, '/earendil/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        # 1-second timer to print status to the screen
        self.create_timer(1.0, self.print_status)
        self.get_logger().info('Hardware checker started. Listening for data...\n')

    def scan_cb(self, msg):
        self.last_msg_times['Lidar (/scan)'] = self.get_clock().now().nanoseconds / 1e9

    def ir_cb(self, msg):
        self.last_msg_times['Infrared (/ir_top)'] = self.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg):
        self.last_msg_times['Motor/Encoder (/odom)'] = self.get_clock().now().nanoseconds / 1e9

    def cam_cb(self, msg):
        self.last_msg_times['Camera (/aruco_visible)'] = self.get_clock().now().nanoseconds / 1e9
        
    def imu_cb(self, msg):
        self.last_msg_times['IMU (/earendil/imu/data_raw)'] = self.get_clock().now().nanoseconds / 1e9
        
    def mag_cb(self, msg):
        self.last_msg_times['Heading (/earendil/heading/deg)'] = self.get_clock().now().nanoseconds / 1e9
        
    def gps_cb(self, msg):
        self.last_msg_times['GPS (/gps/fix)'] = self.get_clock().now().nanoseconds / 1e9

    def print_status(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Clear terminal screen (for better readability)
        print("\033[H\033[J", end="") # ANSI escape code to clear screen
        print("="*50)
        print(" HARDWARE STATUS CHECK ".center(50, "="))
        print("="*50)

        all_ok = True

        for name, last_time in self.last_msg_times.items():
            if last_time == 0.0:
                print(f"[\033[91mERROR\033[0m] {name:25} -> No data received!")
                all_ok = False
            else:
                elapsed = current_time - last_time
                if elapsed > 1.5:
                    print(f"[\033[93mWARNING\033[0m] {name:25} -> Data stream stopped ({elapsed:.1f}s ago)")
                    all_ok = False
                else:
                    print(f"[\033[92m ACTIVE \033[0m] {name:25} -> Delay: {elapsed:.2f}s")

        print("="*50)
        if all_ok:
            print(" RESULT: All hardware is functioning properly! \n")
        else:
            print(" RESULT: There are issues or disconnections in some sensors. \n")

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
