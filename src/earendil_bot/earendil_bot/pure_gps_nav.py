#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class PureGpsNav(Node):
    def __init__(self):
        super().__init__('pure_gps_nav')
        
        # TARGET GPS COORDINATES (Change these!)
        self.declare_parameter('target_lat', 39.925050)
        self.declare_parameter('target_lon', 32.836956)
        
        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        
        self.current_lat = None
        self.current_lon = None
        
        self.prev_lat = None
        self.prev_lon = None
        self.current_heading = 0.0 # Radians (0 is North)
        
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.sub = self.create_subscription(NavSatFix, '/gps/raw_fix', self.gps_cb, 10)
        
        self.timer = self.create_timer(0.5, self.control_loop) # 2 Hz loop
        self.get_logger().info(f"Pure GPS Nav started. Target: {self.target_lat}, {self.target_lon}")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        # Returns distance in meters
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Returns bearing in radians (0 = North, positive = East)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
        return math.atan2(x, y)

    def gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warning("NO GPS FIX!")
            return
            
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Calculate our heading based on movement
        if self.prev_lat is not None and self.prev_lon is not None:
            dist_moved = self.haversine_distance(self.prev_lat, self.prev_lon, self.current_lat, self.current_lon)
            if dist_moved > 0.5: # Only update heading if we moved at least 0.5 meters (avoids noise)
                self.current_heading = self.calculate_bearing(self.prev_lat, self.prev_lon, self.current_lat, self.current_lon)
                self.prev_lat = self.current_lat
                self.prev_lon = self.current_lon
        else:
            self.prev_lat = self.current_lat
            self.prev_lon = self.current_lon

    def control_loop(self):
        if self.current_lat is None:
            return
            
        dist_to_target = self.haversine_distance(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        target_bearing = self.calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        
        msg = Twist()
        
        if dist_to_target < 2.0:
            self.get_logger().info("ARRIVED AT TARGET!")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        # Calculate heading error
        heading_error = target_bearing - self.current_heading
        # Normalize to -pi to pi
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        
        self.get_logger().info(f"Dist: {dist_to_target:.1f}m | Heading Err: {math.degrees(heading_error):.1f}deg")
        
        # Simple Proportional Control
        msg.linear.x = 0.8 # Constant forward speed
        
        if abs(heading_error) > 0.3: # ~17 degrees
            # Turn aggressively towards target
            msg.angular.z = 1.0 if heading_error > 0 else -1.0
            # Slow down forward speed while turning hard
            msg.linear.x = 0.4
        else:
            # Minor corrections
            msg.angular.z = 0.5 * heading_error
            
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
