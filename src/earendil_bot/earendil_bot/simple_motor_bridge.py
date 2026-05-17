#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SimpleMotorBridge(Node):
    def __init__(self):
        super().__init__('simple_motor_bridge')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        self.last_cmd = None
        
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
        
        cmd = "dur"
        
        # Prioritize rotation if we are turning
        if abs(w) > 0.2:
            if w > 0:  # Turn Left (Positive angular.z)
                cmd = "sol_hizli" if abs(w) >= 0.8 else "sol_yavas"
            else:      # Turn Right (Negative angular.z)
                cmd = "sag_hizli" if abs(w) >= 0.8 else "sag_yavas"
                
        # Otherwise, go straight or backward
        elif abs(v) > 0.1:
            if v > 0:  # Forward (Positive linear.x)
                cmd = "ileri_hizli" if abs(v) >= 0.6 else "ileri_yavas"
            else:      # Backward (Negative linear.x)
                cmd = "geri_hizli" if abs(v) >= 0.6 else "geri_yavas"
                
        else:
            cmd = "dur"
            
        # Only send if the command changed (prevents serial spam and buffer overflow)
        if cmd != self.last_cmd:
            command_str = cmd + "\n"
            self.serial.write(command_str.encode('utf-8'))
            self.last_cmd = cmd
            self.get_logger().info(f"Sent string command to Arduino: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorBridge()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
