#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import socket
import json
import threading
import time

class ArucoReceiverNode(Node):
    def __init__(self):
        super().__init__('aruco_receiver')
        
        # ROS 2 Parameters
        self.declare_parameter('jetson_ip', '192.168.1.101')   # Jetson IP
        self.declare_parameter('port_recv', 5005)              # Pi 5 receives data here
        self.declare_parameter('port_send', 5006)              # Pi 5 sends commands here
        self.declare_parameter('timeout', 1.5)                 # Network timeout in seconds
        
        self.jetson_ip = self.get_parameter('jetson_ip').value
        self.port_recv = self.get_parameter('port_recv').value
        self.port_send = self.get_parameter('port_send').value
        self.timeout = self.get_parameter('timeout').value
        
        # Publishers
        self.midpoint_pub = self.create_publisher(Point, '/aruco_midpoint', 10)
        self.visible_pub = self.create_publisher(Bool, '/aruco_visible', 10)
        
        # State variables
        self.last_packet_time = 0.0
        self.is_running = True
        
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
        
        self.get_logger().info(f"Aruco Receiver started. Listening on UDP port {self.port_recv}.")
        self.get_logger().info(f"Sending START commands to Jetson at {self.jetson_ip}:{self.port_send}")

    def _send_heartbeat(self):
        """Sends START command to Jetson to keep perception session active."""
        if not self.is_running:
            return
        try:
            self.sock_send.sendto(b"START", (self.jetson_ip, self.port_send))
        except Exception as e:
            self.get_logger().error(f"Failed to send START heartbeat: {e}")

    def stop_jetson_perception(self):
        """Sends a STOP command to Jetson to shut down its camera pipeline gracefully."""
        self.get_logger().info("Shutting down Aruco Receiver. Sending STOP command to Jetson...")
        self.is_running = False
        try:
            # Send STOP command multiple times to ensure receipt over UDP
            for _ in range(3):
                self.sock_send.sendto(b"STOP", (self.jetson_ip, self.port_send))
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to send STOP command: {e}")
        finally:
            self.sock_send.close()

    def _socket_listener(self):
        """Background thread listening to UDP port for detection data from Jetson."""
        sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock_recv.bind(('0.0.0.0', self.port_recv))
            sock_recv.settimeout(0.5) # Socket timeout for checking loop run status
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket on port {self.port_recv}: {e}")
            return

        while self.is_running and rclpy.ok():
            try:
                data, addr = sock_recv.recvfrom(1024)
                message = data.decode('utf-8')
                parsed_data = json.loads(message)
                
                # Extract values from JSON
                visible = parsed_data.get("visible", False)
                angle = parsed_data.get("angle", 0.0)
                distance = parsed_data.get("distance", 0.0)
                x_offset = parsed_data.get("x_offset", 0.0)
                
                # Update watchdog time
                self.last_packet_time = time.time()
                
                # Publish std_msgs/msg/Bool
                visible_msg = Bool()
                visible_msg.data = visible
                self.visible_pub.publish(visible_msg)
                
                # Publish geometry_msgs/msg/Point
                midpoint_msg = Point()
                midpoint_msg.x = float(angle)
                midpoint_msg.y = float(x_offset)
                midpoint_msg.z = float(distance)
                self.midpoint_pub.publish(midpoint_msg)
                
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
        if self.last_packet_time > 0:
            elapsed = time.time() - self.last_packet_time
            if elapsed > self.timeout:
                self.get_logger().warn(
                    f"No packets received from Jetson for {elapsed:.1f}s. Assuming disconnected. Publishing visible=False.",
                    throttle_duration_sec=3.0
                )
                
                visible_msg = Bool()
                visible_msg.data = False
                self.visible_pub.publish(visible_msg)
                
                midpoint_msg = Point()
                midpoint_msg.x = 0.0
                midpoint_msg.y = 0.0
                midpoint_msg.z = 0.0
                self.midpoint_pub.publish(midpoint_msg)
                
                # Reset packet timer to avoid duplicate warning logs
                self.last_packet_time = 0.0

    def destroy_node(self):
        self.is_running = False
        if self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
