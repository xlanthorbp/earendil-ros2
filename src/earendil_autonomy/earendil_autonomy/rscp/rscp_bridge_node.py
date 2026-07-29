#!/usr/bin/env python3
# RSCP ROS 2 Köprü Düğümü (Bridge Node)
# Hakemlerden RSCP (RS-232 COBS/Protobuf) üzerinden gelen istekleri ROS 2 topic'lerine dönüştürür.
# Robotun yanıtlarını (ACK, TaskFinished, GPSCoordinate, Distance, RoverStatus) hakem modülüne gönderir.

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int32, Float32, Float64, String
from sensor_msgs.msg import NavSatFix

from earendil_autonomy.rscp.rscp_serial_handler import RSCPSerialHandler
from earendil_autonomy.rscp.rscp_protobuf.rscp_protobuf import (
    RequestEnvelope, ResponseEnvelope, Acknowledge, TaskFinished,
    GPSCoordinate, RoverStatus, RoverState, BatteryState
)


class RSCPBridgeNode(Node):
    def __init__(self):
        super().__init__('rscp_bridge_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('status_rate_hz', 1.0) # Periyodik durum yayını (Max 1Hz)
        self.declare_parameter('battery_default_voltage', 12.6)
        self.declare_parameter('battery_default_current', 1.5)
        self.declare_parameter('battery_default_soc', 0.9)
        self.declare_parameter('ack_delay_s', 0.05)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.status_rate_hz = float(self.get_parameter('status_rate_hz').value)
        self.battery_default_voltage = float(self.get_parameter('battery_default_voltage').value)
        self.battery_default_current = float(self.get_parameter('battery_default_current').value)
        self.battery_default_soc = float(self.get_parameter('battery_default_soc').value)
        self.ack_delay_s = float(self.get_parameter('ack_delay_s').value)

        # ROS 2 Command Publishers (Hakem ➔ Robot - Atomic Messages)
        self.pub_stage = self.create_publisher(Int32, '/rscp/command/set_stage', 10)
        self.pub_arm = self.create_publisher(Bool, '/rscp/command/arm', 10)
        self.pub_search_area = self.create_publisher(NavSatFix, '/rscp/command/search_area', 10)
        self.pub_nav_gps = self.create_publisher(NavSatFix, '/rscp/command/navigate_gps', 10)
        self.pub_start_expl = self.create_publisher(Empty, '/rscp/command/start_exploration', 10)

        # ROS 2 Feedback Subscribers (Robot ➔ Hakem)
        self.create_subscription(Empty, '/rscp/feedback/ack', self.on_feedback_ack, 10)
        self.create_subscription(Empty, '/rscp/feedback/task_finished', self.on_feedback_task_finished, 10)
        self.create_subscription(NavSatFix, '/rscp/feedback/gps_coordinate', self.on_feedback_gps_coord, 10)
        self.create_subscription(Float64, '/rscp/feedback/distance', self.on_feedback_distance, 10)

        # Sensor Subscriptions for Periodic RoverStatus Telemetry
        self.current_lat = None
        self.current_lon = None
        self.current_alt = 0.0
        self.current_heading = 0.0
        self.is_armed = False
        self.mode_str = "disarm"

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Float64, '/earendil/heading/deg', self.heading_cb, 10)
        self.create_subscription(Bool, '/rscp/command/arm', self.arm_state_cb, 10)
        self.create_subscription(String, '/earendil/control/mode_request', self.mode_request_cb, 10)

        # Serial Handler setup
        self.serial_handler = RSCPSerialHandler(
            port=self.port,
            baudrate=self.baudrate,
            on_request_cb=self.on_rscp_request,
            logger=self.get_logger()
        )
        self.serial_handler.start()

        # Status Telemetry Timer (1Hz)
        if self.status_rate_hz > 0:
            period = 1.0 / self.status_rate_hz
            self.create_timer(period, self.publish_status_telemetry)

        self.get_logger().info(f"RSCP Bridge Node active on {self.port}.")

    def arm_state_cb(self, msg: Bool):
        self.is_armed = msg.data

    def mode_request_cb(self, msg: String):
        self.mode_str = msg.data.strip().lower()

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def heading_cb(self, msg: Float64):
        self.current_heading = msg.data

    def on_rscp_request(self, req: RequestEnvelope):
        """Called when a raw Protobuf RequestEnvelope packet is received from RSCP client module."""
        req_type = req.WhichOneof('request')
        self.get_logger().info(f"[RSCP IN] Received Request: {req_type}")

        # 1. SetStage
        if req_type == 'set_stage':
            msg = Int32()
            msg.data = req.set_stage.value
            self.pub_stage.publish(msg)
            self.send_ack()

        # 2. ArmDisarm
        elif req_type == 'arm_disarm':
            msg = Bool()
            msg.data = req.arm_disarm.value
            self.pub_arm.publish(msg)
            self.is_armed = msg.data
            self.send_ack()

        # 3. SearchArea (Atomic NavSatFix publishing: lat, lon, altitude=radius)
        elif req_type == 'search_area':
            sa = req.search_area
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            msg.latitude = sa.center_coordinate.latitude
            msg.longitude = sa.center_coordinate.longitude
            msg.altitude = float(sa.radius)
            self.pub_search_area.publish(msg)
            self.send_ack()

        # 4. NavigateToGPS (Atomic NavSatFix publishing)
        elif req_type == 'navigate_to_gps':
            nav = req.navigate_to_gps
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            msg.latitude = nav.coordinate.latitude
            msg.longitude = nav.coordinate.longitude
            msg.altitude = float(nav.coordinate.altitude)
            self.pub_nav_gps.publish(msg)
            self.send_ack()

        # 5. StartExploration
        elif req_type == 'start_exploration':
            self.pub_start_expl.publish(Empty())
            self.send_ack()

    # Feedback Callbacks (Robot ➔ Hakem)

    def send_ack(self):
        if self.ack_delay_s > 0:
            import time
            time.sleep(self.ack_delay_s)
        resp = ResponseEnvelope()
        resp.acknowledge.CopyFrom(Acknowledge())
        self.serial_handler.send_response(resp)
        self.get_logger().info("[RSCP OUT] Sent Acknowledge")

    def on_feedback_ack(self, msg: Empty):
        self.send_ack()

    def on_feedback_task_finished(self, msg: Empty):
        resp = ResponseEnvelope()
        resp.task_finished.CopyFrom(TaskFinished())
        self.serial_handler.send_response(resp)
        self.get_logger().info("[RSCP OUT] Sent TaskFinished")

    def on_feedback_gps_coord(self, msg: NavSatFix):
        resp = ResponseEnvelope()
        resp.gps_coordinate.latitude = msg.latitude
        resp.gps_coordinate.longitude = msg.longitude
        resp.gps_coordinate.altitude = float(msg.altitude)
        self.serial_handler.send_response(resp)
        self.get_logger().info(f"[RSCP OUT] Sent GPSCoordinate: ({msg.latitude:.8f}, {msg.longitude:.8f})")

    def on_feedback_distance(self, msg: Float64):
        resp = ResponseEnvelope()
        resp.distance = float(msg.data)
        self.serial_handler.send_response(resp)
        self.get_logger().info(f"[RSCP OUT] Sent Distance: {msg.data:.2f}m")

    def publish_status_telemetry(self):
        if self.current_lat is None or self.current_lon is None:
            return

        resp = ResponseEnvelope()
        status = RoverStatus()
        
        # Dynamically evaluate state (MANUAL, AUTONOMOUS, DISARMED)
        if self.mode_str == 'manual':
            status.state = RoverState.MANUAL
        elif self.is_armed or self.mode_str in ('auto', 'autonomous'):
            status.state = RoverState.AUTONOMOUS
        else:
            status.state = RoverState.DISARMED

        status.coordinate.latitude = self.current_lat
        status.coordinate.longitude = self.current_lon
        status.coordinate.altitude = float(self.current_alt)
        status.heading = float(self.current_heading)

        # Battery parameterized values
        status.battery_state.voltage = self.battery_default_voltage
        status.battery_state.current = self.battery_default_current
        status.battery_state.state_of_charge = self.battery_default_soc

        resp.rover_status.CopyFrom(status)
        self.serial_handler.send_response(resp)

    def destroy_node(self):
        self.serial_handler.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RSCPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
