"""Single-owner transport (Serial or TCP) for the Earendil STM32H723 terminal link."""

from __future__ import annotations

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:
    serial = None


class H7SerialNode(Node):
    """Own the H7 serial/TCP transport and expose line RX plus validated command TX."""

    def __init__(self) -> None:
        super().__init__("h7_serial_node")

        self.declare_parameter("transport_type", "serial")  # "serial" or "tcp"
        self.declare_parameter("serial_device", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("tcp_host", "127.0.0.1")
        self.declare_parameter("tcp_port", 5000)
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("poll_period_s", 0.005)
        self.declare_parameter("heartbeat_enabled", True)
        self.declare_parameter("heartbeat_period_s", 0.5)
        self.declare_parameter("max_line_length", 1024)
        self.declare_parameter("max_command_length", 128)
        self.declare_parameter("send_safety_stop_on_shutdown", True)

        self._transport_type = str(self.get_parameter("transport_type").value).lower()
        self._device = str(self.get_parameter("serial_device").value)
        self._baud = int(self.get_parameter("baud_rate").value)
        self._tcp_host = str(self.get_parameter("tcp_host").value)
        self._tcp_port = int(self.get_parameter("tcp_port").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        self._heartbeat_enabled = bool(self.get_parameter("heartbeat_enabled").value)
        self._heartbeat_period = float(self.get_parameter("heartbeat_period_s").value)
        self._max_line_length = int(self.get_parameter("max_line_length").value)
        self._max_command_length = int(self.get_parameter("max_command_length").value)
        self._send_safety_stop = bool(
            self.get_parameter("send_safety_stop_on_shutdown").value
        )

        self._rx_pub = self.create_publisher(String, "h7/rx_line", 100)
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._connected_pub = self.create_publisher(
            Bool, "h7/connected", connection_qos
        )
        self.create_subscription(String, "h7/command", self._command_callback, 20)

        self._handle = None  # serial.Serial or socket.socket instance
        self._handle_lock = threading.RLock()
        self._rx_buffer = bytearray()
        self._last_connect_attempt = 0.0
        self._last_heartbeat = 0.0
        self._connected_state: bool | None = None
        self._closing = False

        poll_period = max(0.001, float(self.get_parameter("poll_period_s").value))
        self.create_timer(poll_period, self._poll)
        self._publish_connected(False)

        if self._transport_type == "serial" and serial is None:
            self.get_logger().fatal("pyserial module missing: sudo apt install python3-serial")
        else:
            if self._transport_type == "tcp":
                self.get_logger().info(
                    f"H7 transport preparing (TCP): {self._tcp_host}:{self._tcp_port}"
                )
            else:
                self.get_logger().info(
                    f"H7 transport preparing (Serial): {self._device} @ {self._baud}"
                )

    def _publish_connected(self, value: bool) -> None:
        if self._connected_state == value:
            return
        self._connected_state = value
        msg = Bool()
        msg.data = value
        self._connected_pub.publish(msg)

    def _try_connect(self) -> None:
        if self._closing:
            return
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_period:
            return
        self._last_connect_attempt = now

        new_handle = None
        if self._transport_type == "tcp":
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((self._tcp_host, self._tcp_port))
                sock.setblocking(False)
                new_handle = sock
            except (OSError, socket.error) as exc:
                self.get_logger().warn(
                    f"H7 TCP connection failed ({self._tcp_host}:{self._tcp_port}): {exc}",
                    throttle_duration_sec=5.0,
                )
                self._publish_connected(False)
                return
        else:
            if serial is None:
                return
            try:
                ser = serial.Serial(
                    port=self._device,
                    baudrate=self._baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0,
                    write_timeout=0.2,
                    exclusive=True,
                )
                ser.reset_input_buffer()
                new_handle = ser
            except (OSError, serial.SerialException) as exc:
                self.get_logger().warn(
                    f"H7 serial port could not be opened ({self._device}): {exc}",
                    throttle_duration_sec=5.0,
                )
                self._publish_connected(False)
                return

        with self._handle_lock:
            self._handle = new_handle
        self._rx_buffer.clear()
        self._last_heartbeat = 0.0
        self._publish_connected(True)
        self.get_logger().info(
            f"STM32H723 transport link established ({self._transport_type.upper()})"
        )

    def _disconnect(self, reason: str) -> None:
        with self._handle_lock:
            old_handle = self._handle
            self._handle = None
        if old_handle is not None:
            try:
                old_handle.close()
            except Exception:
                pass
        self._rx_buffer.clear()
        self._publish_connected(False)
        self.get_logger().error(f"STM32H723 transport link disconnected: {reason}")

    def _write_command(self, command: str, *, internal: bool = False) -> bool:
        clean = command.strip()
        if not clean:
            return False
        if "\r" in clean or "\n" in clean:
            if not internal:
                self.get_logger().warn("Rejected H7 command containing newline")
            return False
        if len(clean.encode("utf-8")) > self._max_command_length:
            if not internal:
                self.get_logger().warn("Rejected H7 command exceeding max length")
            return False

        with self._handle_lock:
            handle = self._handle
            if handle is None:
                if not internal:
                    self.get_logger().warn(
                        "H7 not connected; dropped command without queuing"
                    )
                return False
            try:
                payload = (clean + "\r\n").encode("utf-8")
                if self._transport_type == "tcp":
                    handle.sendall(payload)
                else:
                    written = handle.write(payload)
                    handle.flush()
                    if written != len(payload):
                        raise serial.SerialTimeoutException(
                            f"Incomplete serial write: {written}/{len(payload)}"
                        )
            except Exception as exc:
                self._disconnect(str(exc))
                return False
        return True

    def _command_callback(self, msg: String) -> None:
        self._write_command(msg.data)

    def _emit_complete_lines(self) -> None:
        while b"\n" in self._rx_buffer:
            raw, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            line = raw.rstrip(b"\r").decode("utf-8", errors="replace").strip()
            if not line:
                continue
            msg = String()
            msg.data = line
            self._rx_pub.publish(msg)

        if len(self._rx_buffer) > self._max_line_length:
            self.get_logger().warn("H7 RX buffer exceeded max length; cleared")
            self._rx_buffer.clear()

    def _poll(self) -> None:
        if self._handle is None:
            self._try_connect()
            return

        chunk = b""
        try:
            with self._handle_lock:
                handle = self._handle
                if handle is None:
                    return
                if self._transport_type == "tcp":
                    try:
                        chunk = handle.recv(4096)
                        if not chunk:
                            self._disconnect("Remote TCP socket closed connection")
                            return
                    except (BlockingIOError, socket.error) as se:
                        if isinstance(se, BlockingIOError) or getattr(se, 'errno', None) in (socket.EAGAIN, socket.EWOULDBLOCK):
                            chunk = b""
                        else:
                            self._disconnect(str(se))
                            return
                else:
                    waiting = handle.in_waiting
                    chunk = handle.read(min(max(waiting, 1), 4096)) if waiting else b""
        except Exception as exc:
            self._disconnect(str(exc))
            return

        if chunk:
            self._rx_buffer.extend(chunk)
            self._emit_complete_lines()

        now = time.monotonic()
        if (
            self._heartbeat_enabled
            and now - self._last_heartbeat >= self._heartbeat_period
        ):
            if self._write_command("hb", internal=True):
                self._last_heartbeat = now

    def close(self) -> None:
        self._closing = True
        if self._send_safety_stop and self._handle is not None:
            self._write_command("stop", internal=True)
            self._write_command("mode disarm", internal=True)
        with self._handle_lock:
            handle = self._handle
            self._handle = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        self._publish_connected(False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H7SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
