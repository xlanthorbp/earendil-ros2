#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class SimpleMotorBridge(Node):
    def __init__(self):
        super().__init__('simple_motor_bridge')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.6) # meters
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('speed_to_pwm_ratio', 100.0) # 1.0 m/s = 100 PWM
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.ratio = self.get_parameter('speed_to_pwm_ratio').value
        
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial = None
            
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

    def cmd_cb(self, msg: Twist):
        if not self.serial:
            return
            
        v = msg.linear.x
        w = msg.angular.z
        
        # Open-loop differential drive kinematics
        # v_L = v - (w * d / 2), v_R = v + (w * d / 2)
        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)
        
        # Convert m/s to PWM
        pwm_l = int(v_l * self.ratio)
        pwm_r = int(v_r * self.ratio)
        
        # Clamp to max_pwm
        pwm_l = max(min(pwm_l, self.max_pwm), -self.max_pwm)
        pwm_r = max(min(pwm_r, self.max_pwm), -self.max_pwm)
        
        # Send to Arduino
        cmd = f"m {pwm_l} {pwm_r}\n"
        self.serial.write(cmd.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorBridge()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
