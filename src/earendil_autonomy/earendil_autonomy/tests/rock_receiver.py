#!/usr/bin/env python3
# Bu script Raspberry Pi 5 üzerinde çalışmaktadır.
# Jetson Nano ile ethernet kablosu üzerinden kurulan yerel ağ (LAN) bağlantısı vasıtasıyla haberleşir.
# Jetson Nano üzerinde bir systemd servisi (service) olarak çalışan algılama scriptinden UDP soketleri
# üzerinden gelen tespit verilerini (taş/rock koordinat ve genişlik bilgilerini) dinler/alır ve bunları ROS 2 topic'lerine publish eder.
# (Not: earendil_bot paketindeki genel tüm scriptler Raspberry Pi üzerinden çalışmaktadır.
#  Sadece earendil_bot/scripts/ klasöründekiler hariçtir; oradaki kodlar örnek/test kodlarıdır.)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32
import socket
import json
import threading
import time

class RockReceiverNode(Node):
    def __init__(self):
        super().__init__('rock_receiver')
        
        # ROS 2 Parameters
        self.declare_parameter('jetson_ip', '192.168.1.101')   # Jetson IP
        self.declare_parameter('port_recv', 5007)              # Pi 5 receives data here
        self.declare_parameter('port_send', 5008)              # Pi 5 sends commands here
        self.declare_parameter('timeout', 1.5)                 # Network timeout in seconds
        self.declare_parameter('detection_confidence_frames', 3)
        self.declare_parameter('max_valid_detection_dist', 8.0)
        self.declare_parameter('initial_stage', 1)
        self.declare_parameter('active_stage', 2)
        self.declare_parameter('auto_shutdown_after_stage', True)

        self.jetson_ip = self.get_parameter('jetson_ip').value
        self.port_recv = self.get_parameter('port_recv').value
        self.port_send = self.get_parameter('port_send').value
        self.timeout = self.get_parameter('timeout').value
        self.detection_confidence_frames = int(self.get_parameter('detection_confidence_frames').value)
        self.max_valid_detection_dist = float(self.get_parameter('max_valid_detection_dist').value)
        self.initial_stage = int(self.get_parameter('initial_stage').value)
        self.active_stage = int(self.get_parameter('active_stage').value)
        self.auto_shutdown_after_stage = bool(self.get_parameter('auto_shutdown_after_stage').value)
        
        # State variables
        self.current_stage = self.initial_stage
        self.is_enabled = (self.current_stage == self.active_stage)
        self.last_packet_time = 0.0
        self.is_running = True
        self.is_shutting_down = False
        
        # Publishers & Subscriptions
        self.point_pub = self.create_publisher(Point, '/rock_point', 10)
        self.visible_pub = self.create_publisher(Bool, '/rock_visible', 10)
        
        self.stage_sub = self.create_subscription(Int32, '/rscp/command/set_stage', self._stage_cb, 10)
        self.enable_sub = self.create_subscription(Bool, '/rock_receiver/enable', self._enable_cb, 10)
        
        # Socket setup for sending commands (heartbeats)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Start socket receiver thread
        self.recv_thread = threading.Thread(target=self._socket_listener, daemon=True)
        self.recv_thread.start()
        
        # Timer for sending periodic START heartbeats to Jetson (1 Hz / every 1.0 second)
        self.heartbeat_timer = self.create_timer(1.0, self._send_heartbeat)
        
        # Watchdog timer to check for packet timeout (10 Hz / every 0.1 second)
        self.watchdog_timer = self.create_timer(0.1, self._check_timeout)
        
        # ROS 2 shutdown hook: send STOP command to Jetson for clean exit
        rclpy.get_default_context().on_shutdown(self.stop_jetson_perception)
        
        self.get_logger().info(
            f"Rock Receiver initialized (Initial Stage: {self.initial_stage}, Active Stage: {self.active_stage}, Enabled: {self.is_enabled}). "
            f"Listening UDP port {self.port_recv}."
        )

    def _stage_cb(self, msg: Int32):
        new_stage = msg.data
        self.current_stage = new_stage
        self.get_logger().info(f"Rock Receiver stage update received: STAGE {new_stage}")
        
        if new_stage == self.active_stage:
            if not self.is_enabled:
                self.get_logger().info(f"🟢 STAGE {new_stage} activated! Rock receiver enabled -> sending START to Jetson.")
                self.is_enabled = True
        elif new_stage > self.active_stage and self.auto_shutdown_after_stage:
            self.get_logger().warn(f"🛑 STAGE {new_stage} reached (> Stage {self.active_stage}). Shutting down Rock Receiver node cleanly!")
            self._shutdown_and_exit()
        else:
            if self.is_enabled:
                self.get_logger().info(f"⚪ STAGE {new_stage} is not active stage. Disabling Rock receiver -> sending STOP to Jetson.")
                self.is_enabled = False
                self.stop_jetson_perception()
                self._publish_inactive()

    def _enable_cb(self, msg: Bool):
        if msg.data and not self.is_enabled:
            self.get_logger().info("🟢 Rock receiver manually enabled.")
            self.is_enabled = True
        elif not msg.data and self.is_enabled:
            self.get_logger().info("⚪ Rock receiver manually disabled.")
            self.is_enabled = False
            self.stop_jetson_perception()
            self._publish_inactive()

    def _send_heartbeat(self):
        """Sends START command to Jetson to keep perception session active."""
        if not self.is_running or not self.is_enabled:
            return
        try:
            self.sock_send.sendto(b"START", (self.jetson_ip, self.port_send))
        except Exception as e:
            self.get_logger().error(f"Failed to send START heartbeat: {e}")

    def stop_jetson_perception(self):
        """Sends a STOP command to Jetson to shut down its camera pipeline gracefully."""
        self.get_logger().info("Sending STOP command to Jetson for Rock perception...")
        try:
            # Send STOP command multiple times to ensure receipt over UDP
            for _ in range(3):
                self.sock_send.sendto(b"STOP", (self.jetson_ip, self.port_send))
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP command: {e}")

    def _publish_inactive(self):
        visible_msg = Bool()
        visible_msg.data = False
        self.visible_pub.publish(visible_msg)
        
        point_msg = Point()
        point_msg.x = 0.0
        point_msg.y = 0.0
        point_msg.z = 0.0
        self.point_pub.publish(point_msg)

    def _shutdown_and_exit(self):
        if self.is_shutting_down:
            return
        self.is_shutting_down = True
        self.is_enabled = False
        self.stop_jetson_perception()
        self._publish_inactive()
        self.is_running = False
        threading.Thread(target=self._exit_node, daemon=True).start()

    def _exit_node(self):
        time.sleep(0.2)
        try:
            self.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    def _socket_listener(self):
        """Background thread listening to UDP port for detection data from Jetson."""
        sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock_recv.bind(('0.0.0.0', self.port_recv))
            sock_recv.settimeout(0.5) # Socket timeout for checking loop status
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket on port {self.port_recv}: {e}")
            return

        while self.is_running and rclpy.ok():
            try:
                data, addr = sock_recv.recvfrom(1024)
                if not self.is_enabled:
                    continue

                message = data.decode('utf-8')
                parsed_data = json.loads(message)
                
                # Extract values from JSON
                visible = parsed_data.get("visible", False)
                angle = parsed_data.get("angle", 0.0)
                distance = float(parsed_data.get("distance", 0.0))
                x_offset = parsed_data.get("x_offset", 0.0)  # Normalized horizontal offset
                
                # Filter by max valid detection distance
                if distance > self.max_valid_detection_dist:
                    visible = False
                
                # Update watchdog time
                self.last_packet_time = time.time()
                
                # Publish std_msgs/msg/Bool
                visible_msg = Bool()
                visible_msg.data = visible
                self.visible_pub.publish(visible_msg)
                
                # Publish geometry_msgs/msg/Point
                point_msg = Point()
                point_msg.x = float(angle)       # Angle in degrees
                point_msg.y = float(x_offset)    # Normalized offset (-1.0 to 1.0)
                point_msg.z = float(distance)    # Estimated distance (meters)
                self.point_pub.publish(point_msg)
                
                # Log packet details to terminal
                self.get_logger().info(
                    f"Parsed UDP packet -> visible: {visible}, angle: {angle:.2f} deg, dist: {distance:.2f} m, offset: {x_offset:.2f}",
                    throttle_duration_sec=1.0
                )
                
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                self.get_logger().warn("Malformed JSON received from Jetson.")
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"Error in socket receiver thread: {e}")
                time.sleep(0.1)
                
        sock_recv.close()

    def _check_timeout(self):
        """Watchdog to verify connection stability. Publishes visible=False if Jetson goes quiet."""
        if not self.is_enabled:
            return
        if self.last_packet_time > 0:
            elapsed = time.time() - self.last_packet_time
            if elapsed > self.timeout:
                self.get_logger().warn(
                    f"No packets received from Jetson for {elapsed:.1f}s. Assuming disconnected. Publishing visible=False.",
                    throttle_duration_sec=3.0
                )
                
                self._publish_inactive()
                # Reset packet timer to avoid duplicate warning logs
                self.last_packet_time = 0.0

    def destroy_node(self):
        self.is_running = False
        try:
            self.sock_send.close()
        except Exception:
            pass
        if self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RockReceiverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
