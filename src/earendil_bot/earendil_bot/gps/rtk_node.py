#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import serial
import threading
import time

# =======================================================
# RTCM3 PARSER HELPER FUNCTIONS
# =======================================================
def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc

def extract_rtcm3_frames(buffer: bytearray):
    frames = []
    while True:
        start = buffer.find(b"\xD3")
        if start < 0:
            return bytearray(), frames
        if len(buffer) - start < 3:
            return bytearray(buffer[start:]), frames
        if buffer[start + 1] & 0xFC:
            buffer = buffer[start + 1:]
            continue
        length = ((buffer[start + 1] & 0x03) << 8) | buffer[start + 2]
        total_len = 3 + length + 3
        if len(buffer) - start < total_len:
            return bytearray(buffer[start:]), frames
        frame = bytes(buffer[start:start + total_len])
        received_crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        calculated_crc = crc24q(frame[:-3])
        if received_crc == calculated_crc:
            frames.append(frame)
            buffer = buffer[start + total_len:]
        else:
            buffer = buffer[start + 1:]

# =======================================================
# NMEA GGA PARSER
# =======================================================
def nmea2dec(coord: str, direction: str) -> float:
    if not coord:
        raise ValueError("Empty coordinate")
    dot = coord.index(".")
    deg = float(coord[:dot - 2])
    mins = float(coord[dot - 2:])
    dec = deg + mins / 60.0
    if direction in ("S", "W"):
        dec = -dec
    return dec

def parse_gga(line: str):
    if "GGA," not in line:
        return None
    parts = line.split(",")
    if len(parts) < 10:
        return None
    try:
        quality = parts[6]
        lat = None
        lon = None
        altitude = 0.0
        if quality != "0" and parts[2] and parts[4]:
            lat = nmea2dec(parts[2], parts[3])
            lon = nmea2dec(parts[4], parts[5])
            if parts[9]:
                altitude = float(parts[9])
        return {
            "lat": lat,
            "lon": lon,
            "altitude": altitude,
            "quality": int(quality)
        }
    except Exception:
        return None

# =======================================================
# ROS 2 NODE
# =======================================================
class RtkNode(Node):
    def __init__(self):
        super().__init__('rtk_node')
        
        self.declare_parameter('gps_port', '/dev/ttyUSB1')
        self.declare_parameter('radio_port', '/dev/ttyUSB2')
        self.declare_parameter('gps_baud', 460800)
        self.declare_parameter('radio_baud', 57600)

        gps_port = self.get_parameter('gps_port').value
        radio_port = self.get_parameter('radio_port').value
        gps_baud = self.get_parameter('gps_baud').value
        radio_baud = self.get_parameter('radio_baud').value

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        try:
            self.gps_ser = serial.Serial(gps_port, gps_baud, timeout=0.05, write_timeout=1)
            self.radio_ser = serial.Serial(radio_port, radio_baud, timeout=0.05, write_timeout=1)
            self.gps_ser.reset_input_buffer()
            self.radio_ser.reset_input_buffer()
            self.get_logger().info(f"Connected to GPS ({gps_port}) and Radio ({radio_port})")
        except Exception as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise SystemExit

        self.stop_event = threading.Event()
        
        self.rtcm_thread = threading.Thread(target=self.radio_to_gps_rtcm, daemon=True)
        self.gps_thread = threading.Thread(target=self.monitor_rover_gps, daemon=True)
        
        self.rtcm_thread.start()
        self.gps_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        time.sleep(0.2)
        if hasattr(self, 'gps_ser') and self.gps_ser.is_open:
            self.gps_ser.close()
        if hasattr(self, 'radio_ser') and self.radio_ser.is_open:
            self.radio_ser.close()
        super().destroy_node()

    def radio_to_gps_rtcm(self):
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                data = self.radio_ser.read(self.radio_ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    buffer, frames = extract_rtcm3_frames(buffer)
                    for frame in frames:
                        self.gps_ser.write(frame)
                        self.gps_ser.flush()
            except Exception as e:
                time.sleep(0.1)

    def monitor_rover_gps(self):
        while not self.stop_event.is_set():
            try:
                raw_line = self.gps_ser.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("ascii", errors="ignore").strip()
                gga = parse_gga(line)
                if gga is None:
                    continue
                
                if gga["lat"] is not None and gga["lon"] is not None:
                    msg = NavSatFix()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps_link'
                    msg.latitude = gga["lat"]
                    msg.longitude = gga["lon"]
                    msg.altitude = gga["altitude"]
                    
                    
                    if gga["quality"] == 4:
                        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                        fix_str = "RTK_FIXED"
                    elif gga["quality"] == 5:
                        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                        fix_str = "RTK_FLOAT"
                    elif gga["quality"] > 0:
                        msg.status.status = NavSatStatus.STATUS_FIX
                        fix_str = "STANDART_GPS"
                    else:
                        msg.status.status = NavSatStatus.STATUS_NO_FIX
                        fix_str = "NO_FIX"
                        
                    msg.status.service = NavSatStatus.SERVICE_GPS
                    self.fix_pub.publish(msg)
                    
                    # Log information to terminal every 2 seconds for the user
                    now = time.time()
                    if not hasattr(self, 'last_log_time') or (now - self.last_log_time) >= 2.0:
                        self.get_logger().info(f"[GPS] Status: {fix_str} | Lat: {gga['lat']:.7f} | Lon: {gga['lon']:.7f} | Alt: {gga['altitude']}m")
                        self.last_log_time = now
                        
            except Exception as e:
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RtkNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
