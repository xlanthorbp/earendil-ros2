import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading
import time

class IrBridge(Node):
    def __init__(self):
        super().__init__('ir_bridge')
        
        # Parametreler
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        # ROS 2 Publisher
        self.pub_top = self.create_publisher(Range, '/ir_top', 10)
        
        # Seri Port Bağlantısı
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"SHARP IR Arduino bağlandı: {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Arduino'ya bağlanılamadı. Cihaz takılı mı?: {e}")
            self.ser = None
            
        if self.ser:
            self.serial_buffer = ""
            self._thread = threading.Thread(target=self._serial_reader, daemon=True)
            self._thread.start()
            
    def _create_range_msg(self, frame_id, dist_cm):
        """Arduino'dan gelen CM verisini ROS 2 Range mesajına (Metre) çevirir"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05 # Lazer gibi ince bir ışın (yaklaşık 3 derece)
        msg.min_range = 0.20 # 20 cm
        msg.max_range = 1.50 # 150 cm (SHARP 2Y0A02 F81 max range)
        
        if dist_cm < 20.0 or dist_cm > 150.0:
            msg.range = -1.0 # Negatif değer sensörün ölçemediğini gösterir
        else:
            msg.range = dist_cm / 100.0 
            
        return msg

    def _serial_reader(self):
        """Arka planda Arduino'dan sürekli veri okuyan döngü"""
        while rclpy.ok():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting).decode('ascii', errors='ignore')
                    self.serial_buffer += chunk
                    
                    # Tam satırları (Enter'a basılmış kısımları) ayır
                    while '\n' in self.serial_buffer:
                        line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Beklenen format: IR,105.5
                        if line.startswith("IR,"):
                            self._parse_ir(line)
                else:
                    time.sleep(0.01) # CPU'yu yormamak için kısa bekleme
                    
            except Exception as e:
                self.get_logger().error(f"Seri port okuma hatası: {e}", throttle_duration_sec=2.0)
                time.sleep(1.0)

    def _parse_ir(self, line):
        """String'i sayılara çevirip yayınlayan fonksiyon"""
        parts = line.split(',')
        if len(parts) == 2:
            try:
                top = float(parts[1])
                # Debug log: Gelen veriyi her saniye terminale yazdır
                self.get_logger().info(f"Arduino'dan gelen ham veri: {top} cm", throttle_duration_sec=1.0)
                self.pub_top.publish(self._create_range_msg('ir_top_link', top))
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
