import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading
import time

class SonarBridge(Node):
    def __init__(self):
        super().__init__('sonar_bridge')
        
        # Parametreler
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        # ROS 2 Publisher'lar
        self.pub_fl = self.create_publisher(Range, '/sonar_front_left', 10)
        self.pub_fr = self.create_publisher(Range, '/sonar_front_right', 10)
        self.pub_rl = self.create_publisher(Range, '/sonar_rear_left', 10)
        self.pub_rr = self.create_publisher(Range, '/sonar_rear_right', 10)
        self.pub_top = self.create_publisher(Range, '/sonar_top', 10)
        
        # Seri Port Bağlantısı
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Sonar Arduino bağlandı: {port} @ {baud}")
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
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26 # HC-SR04 genelde ~15 derecedir (0.26 radyan)
        msg.min_range = 0.02 # 2 cm
        msg.max_range = 4.0  # 4 metre
        
        # 0 döndüyse veya okunamadıysa geçersiz say, değilse metreye çevir
        if dist_cm <= 0:
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
                        
                        # Beklenen format: SONAR,FL,FR,RL,RR,TOP
                        # Örnek: SONAR,15.2,12.5,50.1,51.0,150.0
                        if line.startswith("SONAR,"):
                            self._parse_sonar(line)
                else:
                    time.sleep(0.01) # CPU'yu yormamak için kısa bekleme
                    
            except Exception as e:
                self.get_logger().error(f"Seri port okuma hatası: {e}", throttle_duration_sec=2.0)
                time.sleep(1.0)

    def _parse_sonar(self, line):
        """String'i sayılara çevirip yayınlayan fonksiyon"""
        parts = line.split(',')
        if len(parts) == 6:
            try:
                fl = float(parts[1])
                fr = float(parts[2])
                rl = float(parts[3])
                rr = float(parts[4])
                top = float(parts[5])
                
                self.pub_fl.publish(self._create_range_msg('sonar_front_left_link', fl))
                self.pub_fr.publish(self._create_range_msg('sonar_front_right_link', fr))
                self.pub_rl.publish(self._create_range_msg('sonar_rear_left_link', rl))
                self.pub_rr.publish(self._create_range_msg('sonar_rear_right_link', rr))
                
                # Bizim tunnel_navigator bu topici dinliyor
                self.pub_top.publish(self._create_range_msg('sonar_top_link', top))
                
            except ValueError:
                pass # Anlık bozuk veri gelirse program çökmesin diye yoksay

def main(args=None):
    rclpy.init(args=args)
    node = SonarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
