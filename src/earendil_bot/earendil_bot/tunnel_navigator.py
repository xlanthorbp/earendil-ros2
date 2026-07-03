import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
import math

class TunnelNavigatorNode(Node):
    def __init__(self):
        super().__init__('tunnel_navigator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.top_sonar_sub = self.create_subscription(Range, '/sonar_top', self.top_sonar_callback, 10)
        
        # Tünel yüksekliği hesaplama için
        self.sensor_height_from_ground = 1.0  # metre cinsinden değiştirilebilir sabit
        self.max_tunnel_height = 0.0
        self.tunnel_start_time = None
        
        # Durumlar: 'APPROACHING', 'IN_TUNNEL', 'COMPLETED'
        self.state = 'APPROACHING'
        
        # Parametreler
        self.forward_speed_approach = 0.2
        self.forward_speed_tunnel = 0.3
        self.safety_speed = 0.1
        self.kp = 0.5
        self.safety_threshold = 0.9  # 90 cm
        
        self.get_logger().info('Tünel Navigasyon Düğümü Başlatıldı.')
        self.get_logger().info('Durum: APPROACHING (Tünele Yaklaşılıyor...)')

    def top_sonar_callback(self, msg):
        # Sadece tünel içindeyken veriyi işle
        if self.state == 'IN_TUNNEL' and self.tunnel_start_time is not None:
            # Tünele girdikten sonraki süreyi saniye cinsinden hesapla
            elapsed_time = (self.get_clock().now() - self.tunnel_start_time).nanoseconds / 1e9
            
            # Tünele girer girmez yarım saniye (0.5 sn) bekle
            if elapsed_time > 0.5:
                # msg.range bize HC-SR04'ten gelen doğrudan üst mesafeyi verir
                total_height = msg.range + self.sensor_height_from_ground
                
                # En yüksek değeri kaydet
                if total_height > self.max_tunnel_height:
                    self.max_tunnel_height = total_height

    def get_avg_distance(self, msg, target_angle_deg, window_deg=10.0):
        target_rad = math.radians(target_angle_deg)
        window_rad = math.radians(window_deg)
        
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            # Geçersiz değerleri filtrele (inf, nan, menzil dışı)
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            # Açıyı [-pi, pi] aralığına normalize et
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            # Hedef açıya olan farkı hesapla
            diff = abs(math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad)))
            
            if diff <= window_rad / 2.0:
                valid_ranges.append(r)
                
        if len(valid_ranges) == 0:
            return 6.0 # Eğer veri yoksa güvenli mesafede kabul et
        return sum(valid_ranges) / len(valid_ranges)

    def scan_callback(self, msg):
        left_dist = self.get_avg_distance(msg, 90.0)
        right_dist = self.get_avg_distance(msg, -90.0)
        
        twist = Twist()
        
        if self.state == 'APPROACHING':
            # Tünele girip girmediğimizi kontrol et (her iki taraf da 6m'den yakın olmalı)
            if left_dist < 6.0 and right_dist < 6.0:
                self.get_logger().info(f'Tünel algılandı! Sol: {left_dist:.2f}m, Sağ: {right_dist:.2f}m. Tünel moduna geçiliyor.')
                self.state = 'IN_TUNNEL'
                self.tunnel_start_time = self.get_clock().now() # Tünele giriş anını kaydet
            else:
                self.get_logger().info('Tünel aranıyor, dümdüz ileri gidiliyor...', throttle_duration_sec=2.0)
                twist.linear.x = self.forward_speed_approach
                
        if self.state == 'IN_TUNNEL':
            # Tünelden çıkış kontrolü
            if left_dist >= 6.0 and right_dist >= 6.0:
                self.get_logger().info(f'Tünel bitti! Görev tamamlandı. Ölçülen Maksimum Tünel Yüksekliği: {self.max_tunnel_height:.2f} metre.')
                self.state = 'COMPLETED'
            else:
                # Acil Kaçınma (Safety) Kontrolü
                if left_dist < self.safety_threshold:
                    self.get_logger().warn(f'ACIL DURUM: Sol duvara çok yakın ({left_dist:.2f}m)! Sağa kaçılıyor.')
                    twist.linear.x = self.safety_speed
                    twist.angular.z = -0.8 # Sert sağ
                elif right_dist < self.safety_threshold:
                    self.get_logger().warn(f'ACIL DURUM: Sağ duvara çok yakın ({right_dist:.2f}m)! Sola kaçılıyor.')
                    twist.linear.x = self.safety_speed
                    twist.angular.z = 0.8 # Sert sol
                else:
                    self.get_logger().info('Tünelde ortalanarak ileri gidiliyor...', throttle_duration_sec=2.0)
                    # Oransal (P) Kontrol ile Ortalama
                    error = left_dist - right_dist
                    twist.linear.x = self.forward_speed_tunnel
                    twist.angular.z = self.kp * error
                    
        if self.state == 'COMPLETED':
            # Görev bitince dur
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TunnelNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Kapatırken motorları durdur
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
