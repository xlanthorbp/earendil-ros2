#!/usr/bin/env python3
"""
Pure GPS + IMU Navigation Node (Auto Base Discovery)
------------------------------------------------------
Listens for the base station GPS coordinates over UDP (broadcast),
uses its own GPS for position and IMU for heading,
then rotates the robot to face the base station.

No manual target_lat / target_lon parameters needed!
The base station runs 'base_gps_sender.py' on the laptop.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math
import socket
import threading


class PureGpsNav(Node):
    def __init__(self):
        super().__init__('pure_gps_nav')

        self.declare_parameter('heading_tolerance', 0.15)    # radians (~8.5 degrees)
        self.declare_parameter('udp_port', 5555)             # Must match base_gps_sender.py

        self.heading_tol = self.get_parameter('heading_tolerance').value
        udp_port = self.get_parameter('udp_port').value

        # State
        self.current_lat = None
        self.current_lon = None
        self.imu_heading = None
        self.base_lat = None
        self.base_lon = None

        # Publishers & Subscribers
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(NavSatFix, '/gps/raw_fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Start UDP listener thread for base station coordinates
        self._udp_thread = threading.Thread(target=self._udp_listener, args=(udp_port,), daemon=True)
        self._udp_thread.start()

        # Control loop at 2 Hz
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info(f"GPS+IMU Nav started. Listening for base station on UDP port {udp_port}...")

    # ==================================================
    # UDP Listener (receives base station GPS from laptop)
    # ==================================================
    def _udp_listener(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', port))

        while True:
            try:
                data, addr = sock.recvfrom(256)
                msg = data.decode('ascii', errors='ignore').strip()
                # Expected format: "BASE,39.92505000,32.83695600"
                if msg.startswith('BASE,'):
                    parts = msg.split(',')
                    self.base_lat = float(parts[1])
                    self.base_lon = float(parts[2])
            except Exception:
                pass

    # ==================================================
    # GPS Callback (rover's own GPS)
    # ==================================================
    def gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warning("NO GPS FIX!", throttle_duration_sec=5.0)
            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    # ==================================================
    # IMU Callback — extract yaw (heading) from quaternion
    # ==================================================
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_heading = yaw

    # ==================================================
    # Math helpers
    # ==================================================
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Returns distance in meters between two GPS coordinates."""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Returns bearing in radians (0=North, positive=East)."""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = (math.cos(phi1) * math.sin(phi2) -
             math.sin(phi1) * math.cos(phi2) * math.cos(dlam))
        return math.atan2(x, y)

    # ==================================================
    # Main control loop (runs at 2 Hz)
    # ==================================================
    def control_loop(self):
        msg = Twist()

        # Wait for all three data sources
        if self.base_lat is None:
            self.get_logger().info("Waiting for base station GPS (laptop)...", throttle_duration_sec=3.0)
            return
        if self.current_lat is None:
            self.get_logger().info("Waiting for rover GPS fix...", throttle_duration_sec=3.0)
            return
        if self.imu_heading is None:
            self.get_logger().info("Waiting for IMU data...", throttle_duration_sec=3.0)
            return

        # Calculate distance and bearing to base
        dist = self.haversine_distance(
            self.current_lat, self.current_lon,
            self.base_lat, self.base_lon)
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon,
            self.base_lat, self.base_lon)

        # Calculate heading error
        heading_error = target_bearing - self.imu_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().info(
            f"Base: ({self.base_lat:.6f}, {self.base_lon:.6f}) | "
            f"Dist: {dist:.1f}m | "
            f"IMU: {math.degrees(self.imu_heading):.1f}° | "
            f"Bearing: {math.degrees(target_bearing):.1f}° | "
            f"Error: {math.degrees(heading_error):.1f}°")

        # ROTATE toward base. Stop when aligned.
        if abs(heading_error) > self.heading_tol:
            msg.linear.x = 0.0
            if heading_error > 0:
                msg.angular.z = 0.5   # Turn left
            else:
                msg.angular.z = -0.5  # Turn right
        else:
            self.get_logger().info("ALIGNED WITH BASE! Holding position.")
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PureGpsNav()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
