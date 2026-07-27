#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# RSCP Stage 1 (Anten / Zirve Görevi) için çift halkalı (4x ve 2x yarıçaplı dairesel çevre taraması)
# yaparak GPS altimetre verisinden en yüksek noktayı (zirveyi) bulan ve o noktaya otonom yönelen noddur.

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


def calculate_circle_waypoints(center_lat, center_lon, radius_m, num_points=8):
    """
    Belirtilen merkez enlem/boylam etrafında 'radius_m' metre yarıçaplı
    'num_points' adet eşit aralıklı dairesel waypoint üretir.
    """
    waypoints = []
    # 1 derece enlem ~ 111,139 metre
    lat_deg_per_m = 1.0 / 111139.0
    lon_deg_per_m = 1.0 / (111139.0 * math.cos(math.radians(center_lat)))

    for i in range(num_points):
        angle_rad = (2.0 * math.pi / num_points) * i
        dx = radius_m * math.sin(angle_rad) # Doğu yönü
        dy = radius_m * math.cos(angle_rad) # Kuzey yönü

        wp_lat = center_lat + (dy * lat_deg_per_m)
        wp_lon = center_lon + (dx * lon_deg_per_m)
        waypoints.append((wp_lat, wp_lon))

    return waypoints


class PeakFinderNode(Node):
    def __init__(self):
        super().__init__('peak_finder')

        # Target Coordinates & Parameters
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)
        self.declare_parameter('search_radius', 10.0)    # Toplam arama yarıçapı R (Metre)
        self.declare_parameter('outer_ratio', 0.8)       # Dış halka oranı (4x/5x = %80)
        self.declare_parameter('inner_ratio', 0.4)       # İç halka oranı (2x/5x = %40)
        self.declare_parameter('arrival_radius', 0.6)    # Waypoint'e varış toleransı (Metre)
        self.declare_parameter('heading_tolerance_deg', 8.0) # Derece
        self.declare_parameter('max_linear_x', 0.4)      # İleri maksimum hız
        self.declare_parameter('max_angular_z', 0.6)     # Dönüş maksimum hız
        self.declare_parameter('kp_angular', 2.0)        # Dönüş P-kazancı
        self.declare_parameter('kp_lane', 1.5)           # Şerit takip P-kazancı
        self.declare_parameter('dry_run', False)

        self.target_lat = float(self.get_parameter('target_lat').value)
        self.target_lon = float(self.get_parameter('target_lon').value)
        self.search_radius = float(self.get_parameter('search_radius').value)
        self.outer_ratio = float(self.get_parameter('outer_ratio').value)
        self.inner_ratio = float(self.get_parameter('inner_ratio').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius').value)
        heading_tol_deg = float(self.get_parameter('heading_tolerance_deg').value)
        self.heading_tol = math.radians(heading_tol_deg)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.kp_lane = float(self.get_parameter('kp_lane').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        # State Variables
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.mag_heading = None

        self.last_gps_time = 0.0
        self.last_mag_time = 0.0

        # State Machine:
        # DRIVING_TO_CENTER -> GENERATING_OUTER_CIRCLE -> CIRCLING_OUTER_CIRCLE ->
        # GENERATING_INNER_CIRCLE -> CIRCLING_INNER_CIRCLE -> FINDING_PEAK ->
        # NAVIGATING_TO_PEAK -> PEAK_REACHED
        self.state = 'DRIVING_TO_CENTER'
        self.aligned = False

        # Circle Waypoints
        self.outer_waypoints = []
        self.inner_waypoints = []
        self.current_wp_idx = 0

        # Data collection for altitude analysis: list of (lat, lon, alt)
        self.altitude_records = []

        # Discovered peak coordinates
        self.peak_lat = None
        self.peak_lon = None
        self.peak_alt = -9999.0

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(String, '/earendil/control/command', 10)
        self.peak_pub = self.create_publisher(NavSatFix, '/gps/peak_coordinate', 10)

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.mag_cb, 10)

        # Control Loop 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Peak Finder Node Started.')
        self.get_logger().info(f'Target Center: ({self.target_lat:.6f}, {self.target_lon:.6f}) | Radius: {self.search_radius}m')
        self.get_logger().info('State: DRIVING_TO_CENTER')

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        self.last_gps_time = time.time()

        # Record valid GPS positions during circling phases
        if self.state in ['CIRCLING_OUTER_CIRCLE', 'CIRCLING_INNER_CIRCLE']:
            if msg.status.status >= 0 and self.current_alt is not None:
                self.altitude_records.append((self.current_lat, self.current_lon, self.current_alt))

    def mag_cb(self, msg: Float64):
        # /earendil/heading/deg is in degrees (0-360)
        self.mag_heading = math.radians(msg.data)
        self.last_mag_time = time.time()

    def send_motor_cmd(self, v, w):
        if self.dry_run:
            v = 0.0
            w = 0.0

        msg = String()
        if abs(w) > 0.1:
            pwm = 60 + int((abs(w) / self.max_angular_z) * 30)
            msg.data = f"l {min(90, max(60, pwm))}" if w > 0 else f"r {min(90, max(60, pwm))}"
        elif abs(v) > 0.05:
            pwm = 60 + int((abs(v) / self.max_linear_x) * 30)
            msg.data = f"f {min(90, max(60, pwm))}" if v > 0 else f"b {min(90, max(60, pwm))}"
        else:
            msg.data = "stop"

        self.cmd_pub.publish(msg)

    def drive_towards(self, goal_lat, goal_lon):
        """
        Calculates bearing and distance to goal, and drives the vehicle.
        Returns distance to goal.
        """
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, goal_lat, goal_lon)
        distance = haversine(self.current_lat, self.current_lon, goal_lat, goal_lon)
        error = angle_error_rad(target_bearing, self.mag_heading)

        v = 0.0
        w = 0.0

        # Phase 1: Rotate in place if heading error is large
        if not self.aligned:
            if abs(error) > self.heading_tol:
                w = self.kp_angular * error
                w = max(-self.max_angular_z, min(self.max_angular_z, w))
                v = 0.0
            else:
                self.aligned = True

        # Phase 2: Drive forward with lane keeping
        if self.aligned:
            if abs(error) > self.heading_tol * 3:
                self.aligned = False
                v = 0.0
                w = 0.0
            else:
                v = self.max_linear_x
                w = self.kp_lane * error
                w = max(-self.max_angular_z, min(self.max_angular_z, w))

        self.send_motor_cmd(v, w)
        return distance

    def control_loop(self):
        if self.state == 'PEAK_REACHED':
            return

        if self.target_lat == 0.0 and self.target_lon == 0.0:
            self.get_logger().warn("Target coordinates not specified! (0.0, 0.0). Waiting...", throttle_duration_sec=3.0)
            return

        if self.mag_heading is None or (time.time() - self.last_mag_time > 1.5):
            self.get_logger().warn("Waiting for Magnetometer data (/earendil/heading/deg)...", throttle_duration_sec=2.0)
            self.send_motor_cmd(0.0, 0.0)
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("Waiting for GPS data (/gps/fix)...", throttle_duration_sec=2.0)
            self.send_motor_cmd(0.0, 0.0)
            return

        # STATE 1: Drive to center coordinate
        if self.state == 'DRIVING_TO_CENTER':
            dist = self.drive_towards(self.target_lat, self.target_lon)
            self.get_logger().info(f"[DRIVING_TO_CENTER] Distance to center: {dist:.2f}m", throttle_duration_sec=1.0)
            if dist <= self.arrival_radius:
                self.get_logger().info("Reached search area center! Switching to GENERATING_OUTER_CIRCLE.")
                self.send_motor_cmd(0.0, 0.0)
                self.state = 'GENERATING_OUTER_CIRCLE'

        # STATE 2: Generate 4x (80% radius) circle waypoints
        elif self.state == 'GENERATING_OUTER_CIRCLE':
            r_outer = self.search_radius * self.outer_ratio
            self.outer_waypoints = calculate_circle_waypoints(self.target_lat, self.target_lon, r_outer, num_points=8)
            self.current_wp_idx = 0
            self.aligned = False
            self.get_logger().info(f"Generated 8 outer circle waypoints (radius: {r_outer:.1f}m). Starting CIRCLING_OUTER_CIRCLE.")
            self.state = 'CIRCLING_OUTER_CIRCLE'

        # STATE 3: Drive 360 degrees around outer circle
        elif self.state == 'CIRCLING_OUTER_CIRCLE':
            if self.current_wp_idx < len(self.outer_waypoints):
                wp_lat, wp_lon = self.outer_waypoints[self.current_wp_idx]
                dist = self.drive_towards(wp_lat, wp_lon)
                self.get_logger().info(
                    f"[CIRCLING_OUTER] WP {self.current_wp_idx + 1}/8 | Dist: {dist:.2f}m | Records: {len(self.altitude_records)}",
                    throttle_duration_sec=1.0
                )
                if dist <= self.arrival_radius:
                    self.current_wp_idx += 1
                    self.aligned = False
            else:
                self.get_logger().info("Completed 360-degree outer circle sweep! Switching to GENERATING_INNER_CIRCLE.")
                self.send_motor_cmd(0.0, 0.0)
                self.state = 'GENERATING_INNER_CIRCLE'

        # STATE 4: Generate 2x (40% radius) circle waypoints
        elif self.state == 'GENERATING_INNER_CIRCLE':
            r_inner = self.search_radius * self.inner_ratio
            self.inner_waypoints = calculate_circle_waypoints(self.target_lat, self.target_lon, r_inner, num_points=8)
            self.current_wp_idx = 0
            self.aligned = False
            self.get_logger().info(f"Generated 8 inner circle waypoints (radius: {r_inner:.1f}m). Starting CIRCLING_INNER_CIRCLE.")
            self.state = 'CIRCLING_INNER_CIRCLE'

        # STATE 5: Drive 360 degrees around inner circle
        elif self.state == 'CIRCLING_INNER_CIRCLE':
            if self.current_wp_idx < len(self.inner_waypoints):
                wp_lat, wp_lon = self.inner_waypoints[self.current_wp_idx]
                dist = self.drive_towards(wp_lat, wp_lon)
                self.get_logger().info(
                    f"[CIRCLING_INNER] WP {self.current_wp_idx + 1}/8 | Dist: {dist:.2f}m | Records: {len(self.altitude_records)}",
                    throttle_duration_sec=1.0
                )
                if dist <= self.arrival_radius:
                    self.current_wp_idx += 1
                    self.aligned = False
            else:
                self.get_logger().info("Completed 360-degree inner circle sweep! Switching to FINDING_PEAK.")
                self.send_motor_cmd(0.0, 0.0)
                self.state = 'FINDING_PEAK'

        # STATE 6: Analyze recorded points to find maximum altitude
        elif self.state == 'FINDING_PEAK':
            if not self.altitude_records:
                self.get_logger().warn("No valid altitude records collected! Using target center as peak.")
                self.peak_lat = self.target_lat
                self.peak_lon = self.target_lon
                self.peak_alt = 0.0
            else:
                # Find entry with highest altitude
                best_record = max(self.altitude_records, key=lambda rec: rec[2])
                self.peak_lat = best_record[0]
                self.peak_lon = best_record[1]
                self.peak_alt = best_record[2]

            self.get_logger().info(
                f"🏔️ PEAK IDENTIFIED! Coordinates: ({self.peak_lat:.8f}, {self.peak_lon:.8f}) | "
                f"Max Altitude: {self.peak_alt:.2f}m (from {len(self.altitude_records)} samples)"
            )
            self.aligned = False
            self.state = 'NAVIGATING_TO_PEAK'

        # STATE 7: Drive to the peak coordinates
        elif self.state == 'NAVIGATING_TO_PEAK':
            dist = self.drive_towards(self.peak_lat, self.peak_lon)
            self.get_logger().info(f"[NAVIGATING_TO_PEAK] Distance to peak: {dist:.2f}m", throttle_duration_sec=1.0)
            if dist <= self.arrival_radius:
                self.get_logger().info("🎯 ARRIVED AT PEAK POINT! Stopping vehicle and publishing peak coordinate.")
                self.send_motor_cmd(0.0, 0.0)
                self.state = 'PEAK_REACHED'

                # Publish Peak NavSatFix
                peak_msg = NavSatFix()
                peak_msg.header.stamp = self.get_clock().now().to_msg()
                peak_msg.header.frame_id = 'gps'
                peak_msg.latitude = self.peak_lat
                peak_msg.longitude = self.peak_lon
                peak_msg.altitude = self.peak_alt
                peak_msg.status.status = 2  # STATUS_GBAS_FIX
                self.peak_pub.publish(peak_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PeakFinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.send_motor_cmd(0.0, 0.0)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
