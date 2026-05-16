import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class ArduinoJoyNode(Node):
    def __init__(self):
        super().__init__('arduino_joy_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        # NOTE: Check your Arduino port! It might be /dev/ttyACM0 or /dev/ttyUSB0
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        except serial.serialutil.SerialException:
            self.get_logger().warn("Arduino joystick not found at /dev/ttyUSB0. Running without joystick control.")
            self.serial_port = None

    def timer_callback(self):
        if self.serial_port is not None and self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').rstrip()
                # Parse the comma-separated values from Arduino
                x, y, btn = map(int, line.split(','))
                
                # Arduino analogRead goes from 0 to 1023. 
                # ROS 2 Joy axes should ideally go from -1.0 to 1.0.
                x_mapped = (x - 512) / 512.0
                y_mapped = (y - 512) / 512.0
                
                msg = Joy()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "arduino_joy"
                
                # Assign axes and button
                msg.axes = [x_mapped, y_mapped]
                
                # The button uses pull-up, so 0 is pressed and 1 is unpressed.
                # We invert it (1 - btn) so 1 means pressed in ROS.
                msg.buttons = [1 - btn] 
                
                self.publisher_.publish(msg)
            except ValueError:
                # Ignore malformed serial lines that happen during startup
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port is not None:
            node.serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
