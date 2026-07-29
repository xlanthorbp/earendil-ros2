#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
"""
Heading Test — Magnetometer Based Angle Turn
---------------------------------------------
Subscribes to Magnetometer heading on /mag/heading.
Rotates the vehicle until it reaches the specified target angle (degrees).

Usage:
  ros2 run earendil_autonomy heading_test --ros-args -p target_heading:=180.0
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import math
import time
from earendil_autonomy.gps.gps_math import angle_error_rad, normalize_heading_deg


class HeadingTest(Node):
    def __init__(self):
        super().__init__('heading_test')

        # Target Angle Parameter (degrees: 0.0 - 360.0)
        self.declare_parameter('target_heading', 0.0)

        self.declare_parameter('heading_tolerance_deg', 8.5) # degrees
        self.declare_parameter('turn_speed', 0.5)            # rad/s
        self.declare_parameter('kp_angular', 2.0)            # P-gain for rotation
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('dry_run', False)

        target_heading_param = self.get_parameter('target_heading').value
        target_heading_normalized = normalize_heading_deg(target_heading_param)
        self.target_bearing = math.radians(target_heading_normalized)

        heading_tol_deg = self.get_parameter('heading_tolerance_deg').value
        self.heading_tol = math.radians(heading_tol_deg)
        self.turn_speed = self.get_parameter('turn_speed').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.invert_turn = self.get_parameter('invert_turn').value
        self.dry_run = self.get_parameter('dry_run').value

        self.get_logger().info(f"Target Heading Set To: {target_heading_normalized:.1f}°")
        self.get_logger().info(f"Tolerance: {heading_tol_deg:.1f}°")
        self.get_logger().info(f"Beklenen /earendil/heading/deg yayını için hazırız...")

        # State
        self.mag_heading = None
        self.last_mag_time = 0.0

        # Publisher & Subscriber
        self.pub = self.create_publisher(String, '/earendil/control/command', 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.mag_cb, 10)

        # Control loop 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def mag_cb(self, msg: Float64):
        # /earendil/heading/deg yayını derece cinsindendir.
        self.mag_heading = math.radians(msg.data)
        self.last_mag_time = time.time()

    def send_motor_cmd(self, v, w):
        if self.dry_run:
            v = 0.0
            w = 0.0
            
        msg = String()
        if v == 0.0 and w == 0.0:
            msg.data = "stop"
        elif w > 0:
            pwm = 60 + int((abs(w) / self.turn_speed) * 30)
            msg.data = f"l {min(90, max(60, pwm))}"
        elif w < 0:
            pwm = 60 + int((abs(w) / self.turn_speed) * 30)
            msg.data = f"r {min(90, max(60, pwm))}"
        self.pub.publish(msg)

    def control_loop(self):
        if self.mag_heading is None or (time.time() - self.last_mag_time > 1.5):
            self.get_logger().warn("Magnetometer data (/earendil/heading/deg) bekleniyor veya bağlantı koptu!", throttle_duration_sec=2.0)
            self.send_motor_cmd(0.0, 0.0)
            return

        # Heading error
        error = angle_error_rad(self.target_bearing, self.mag_heading)

        self.get_logger().info(
            f"MAG: {math.degrees(self.mag_heading):.1f}° | "
            f"Target: {math.degrees(self.target_bearing):.1f}° | "
            f"Error: {math.degrees(error):.1f}°",
            throttle_duration_sec=0.5
        )

        angular_vel = 0.0
        if abs(error) > self.heading_tol:
            # P-Controller logic
            angular_vel = self.kp_angular * error
            if angular_vel > self.turn_speed: angular_vel = self.turn_speed
            elif angular_vel < -self.turn_speed: angular_vel = -self.turn_speed
        else:
            self.get_logger().info("ALIGNED WITH TARGET HEADING!", throttle_duration_sec=1.0)
            angular_vel = 0.0

        if self.invert_turn:
            angular_vel = -angular_vel

        self.send_motor_cmd(0.0, angular_vel)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish stop command
        msg = String()
        msg.data = "stop"
        node.pub.publish(msg)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()



