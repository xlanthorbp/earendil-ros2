#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time

class HardwareCheckerNode(Node):
    def __init__(self):
        super().__init__('hardware_checker')
        
        # Son veri alma zamanlarını tutacağımız sözlük
        self.last_msg_times = {
            'Lidar (/scan)': 0.0,
            'Kızılötesi (/ir_top)': 0.0,
            'Motor/Encoder (/odom)': 0.0,
            'Kamera (/aruco_visible)': 0.0,
            'IMU (/imu/data)': 0.0,
            'Manyetometre (/mag/heading)': 0.0,
            'GPS (/gps/fix)': 0.0
        }

        # Abonelikler
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Range, '/ir_top', self.ir_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/aruco_visible', self.cam_cb, 10)
        
        # Eklenen yeni donanımlar (IMU, Mag, GPS)
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import Float32
        
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        # Durumu ekrana basmak için 1 saniyelik zamanlayıcı
        self.create_timer(1.0, self.print_status)
        self.get_logger().info('Donanım kontrolcüsü başlatıldı. Veriler dinleniyor...\n')

    def scan_cb(self, msg):
        self.last_msg_times['Lidar (/scan)'] = time.time()

    def ir_cb(self, msg):
        self.last_msg_times['Kızılötesi (/ir_top)'] = time.time()

    def odom_cb(self, msg):
        self.last_msg_times['Motor/Encoder (/odom)'] = time.time()

    def cam_cb(self, msg):
        self.last_msg_times['Kamera (/aruco_visible)'] = time.time()
        
    def imu_cb(self, msg):
        self.last_msg_times['IMU (/imu/data)'] = time.time()
        
    def mag_cb(self, msg):
        self.last_msg_times['Manyetometre (/mag/heading)'] = time.time()
        
    def gps_cb(self, msg):
        self.last_msg_times['GPS (/gps/fix)'] = time.time()

    def print_status(self):
        current_time = time.time()
        
        # Terminal ekranını temizleyip (görsel olarak daha okunabilir olması için) yazdıralım
        print("\033[H\033[J", end="") # ANSI escape code to clear screen
        print("="*50)
        print(" DONANIM DURUM KONTROLÜ ".center(50, "="))
        print("="*50)

        all_ok = True

        for name, last_time in self.last_msg_times.items():
            if last_time == 0.0:
                print(f"[\033[91mHATA\033[0m] {name:25} -> Hiç veri gelmedi!")
                all_ok = False
            else:
                elapsed = current_time - last_time
                if elapsed > 1.5:
                    print(f"[\033[93mUYARI\033[0m] {name:25} -> Veri akışı durdu ({elapsed:.1f}sn önce)")
                    all_ok = False
                else:
                    print(f"[\033[92m AKTİF \033[0m] {name:25} -> Gecikme: {elapsed:.2f}sn")

        print("="*50)
        if all_ok:
            print(" SONUÇ: Bütün donanımlar tıkır tıkır çalışıyor! \n")
        else:
            print(" SONUÇ: Bazı sensörlerde sorun veya bağlantı kopukluğu var. \n")

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
