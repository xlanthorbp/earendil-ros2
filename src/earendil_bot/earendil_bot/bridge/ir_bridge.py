import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading
import time

class IrBridge(Node):
    def __init__(self):
        super().__init__('ir_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        # ROS 2 Publisher
        self.pub_top = self.create_publisher(Range, '/ir_top', 10)
        
        # Serial Port Connection
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"SHARP IR Arduino connected: {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino. Is it plugged in?: {e}")
            self.ser = None
            
        if self.ser:
            self.serial_buffer = ""
            self._thread = threading.Thread(target=self._serial_reader, daemon=True)
            self._thread.start()
            
    def _create_range_msg(self, frame_id, dist_cm):
        """Converts CM data from Arduino into a ROS 2 Range message (Meters)"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05 # Narrow beam like a laser (approx 3 degrees)
        msg.min_range = 0.35 # 35 cm
        msg.max_range = 1.49 # 149 cm
        
        if dist_cm < 35.0 or dist_cm > 149.0:
            msg.range = -1.0 # Negative value indicates out of range
        else:
            msg.range = dist_cm / 100.0 
            
        return msg

    def _serial_reader(self):
        """Background loop constantly reading data from Arduino"""
        while rclpy.ok():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting).decode('ascii', errors='ignore')
                    self.serial_buffer += chunk
                    
                    # Extract full lines (separated by newline)
                    while '\n' in self.serial_buffer:
                        line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Expected format: IR,105.5
                        if line.startswith("IR,"):
                            self._parse_ir(line)
                else:
                    time.sleep(0.01) # Short sleep to prevent CPU hogging
                    
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}", throttle_duration_sec=2.0)
                time.sleep(1.0)

    def _parse_ir(self, line):
        """Parses the string into numbers and publishes them"""
        parts = line.split(',')
        if len(parts) == 2:
            try:
                top = float(parts[1])
                self.pub_top.publish(self._create_range_msg('ir_top_link', top))
                self.get_logger().info(f"SHARP Height: {top} cm", throttle_duration_sec=0.5)
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = IrBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
