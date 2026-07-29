"""Convert QMC5883L telemetry from the H7 to sensor_msgs/MagneticField."""

from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String

from .telemetry_parser import as_int, parse_record


UT_X100_TO_TESLA = 1.0e-8


class MagnetometerNode(Node):
    def __init__(self) -> None:
        super().__init__("magnetometer_node")
        self.declare_parameter("frame_id", "imu_link")
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._mag_pub = self.create_publisher(
            MagneticField, "imu/magnetic_field", 20
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "imu/magnetometer_diagnostics", 10
        )
        self.create_subscription(String, "h7/rx_line", self._line_callback, 100)

    def _line_callback(self, msg: String) -> None:
        fields = parse_record(msg.data, "MAG_IMU,")
        if fields is None:
            return

        stamp = self.get_clock().now().to_msg()
        ok = as_int(fields, "OK") == 1
        self._publish_diagnostics(fields, stamp, ok)
        if not ok:
            return

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = self._frame_id
        magnetic.magnetic_field.x = as_int(fields, "MX_UTX100") * UT_X100_TO_TESLA
        magnetic.magnetic_field.y = as_int(fields, "MY_UTX100") * UT_X100_TO_TESLA
        magnetic.magnetic_field.z = as_int(fields, "MZ_UTX100") * UT_X100_TO_TESLA
        self._mag_pub.publish(magnetic)

    def _publish_diagnostics(self, fields, stamp, ok: bool) -> None:
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "earendil/qmc5883l"
        status.hardware_id = "STM32H723:I2C1:0x0D"
        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        status.message = "ONLINE" if ok else fields.get("STATE", "INVALID")
        for key in (
            "STATE",
            "AGE_MS",
            "INIT",
            "FOUND",
            "COMM_ERR",
            "DRDY_TOUT",
            "RECOVERY",
            "VERIFY_MM",
        ):
            if key in fields:
                status.values.append(KeyValue(key=key, value=fields[key]))
        array.status.append(status)
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MagnetometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
