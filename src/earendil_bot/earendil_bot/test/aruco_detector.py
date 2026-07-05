#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import cv2
import numpy as np
import math

# Picamera2 import wrapped in try-except in case it's run on a system without it for testing
try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

# =========================
# AYARLAR
# =========================
TAG_SIZE_M = 0.20  # 20x20 cm ArUco tag
HFOV_DEG = 83.0   # IMX219-83 yaklaşık yatay görüş açısı


class RPI_Camera:
    def __init__(self):
        if Picamera2 is None:
            raise ImportError("Picamera2 is not installed or not running on Raspberry Pi")
            
        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={
                "size": (1280, 720),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

    def read(self):
        frame = self.picam2.capture_array()

        if frame is None:
            return False, None

        # Picamera2 RGB verir, OpenCV BGR ister.
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return True, frame

    def release(self):
        self.picam2.stop()


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.midpoint_pub = self.create_publisher(Point, '/aruco_midpoint', 10)
        self.visible_pub = self.create_publisher(Bool, '/aruco_visible', 10)
        
        try:
            self.cap = RPI_Camera()
        except Exception as e:
            self.get_logger().error(f"Kamera baslatilamadi: {e}")
            self.cap = None

        if self.cap:
            # Ilk frame'i okuyup parametreleri hazirla
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Kamera acildi ama ilk frame alinamadi.")
            else:
                self.get_logger().info(f"Kamera calisiyor. Boyut: {frame.shape}")
                self.IMAGE_H, self.IMAGE_W = frame.shape[:2]
                
                self.fx = (self.IMAGE_W / 2) / math.tan(math.radians(HFOV_DEG / 2))
                self.fy = self.fx
                self.cx = self.IMAGE_W / 2
                self.cy = self.IMAGE_H / 2
                
                # ArUco ayarlari
                if hasattr(cv2.aruco, "getPredefinedDictionary"):
                    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
                else:
                    self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

                if hasattr(cv2.aruco, "DetectorParameters"):
                    self.aruco_params = cv2.aruco.DetectorParameters()
                else:
                    self.aruco_params = cv2.aruco.DetectorParameters_create()
                    
                # Timer - saniyede 15 kere (15 Hz) cagir
                self.timer = self.create_timer(1.0 / 15.0, self.process_frame)

    def marker_center(self, corners):
        c = np.mean(corners, axis=0)
        return int(c[0]), int(c[1])

    def marker_area(self, corners):
        return cv2.contourArea(corners.astype(np.float32))

    def distance_from_area(self, area_px):
        if area_px <= 0:
            return None
        side_px = math.sqrt(area_px)
        z_m = self.fx * TAG_SIZE_M / side_px
        return z_m

    def pixel_to_camera_direction(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Kamera goruntusu alinamadi.", throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        visible_msg = Bool()
        visible_msg.data = False
        
        if ids is not None and len(ids) >= 2:
            ids = ids.flatten()
            detected_markers = []

            for i, marker_id in enumerate(ids):
                marker_corners = corners[i][0]
                area_px = self.marker_area(marker_corners)
                z_m = self.distance_from_area(area_px)

                if z_m is None:
                    continue

                u, v = self.marker_center(marker_corners)
                detected_markers.append({
                    "id": int(marker_id),
                    "center": (u, v),
                    "area_px": area_px,
                    "distance_m": z_m
                })

            if len(detected_markers) >= 2:
                # Soldan saga sirala
                detected_markers.sort(key=lambda m: m["center"][0])
                tag1 = detected_markers[0]
                tag2 = detected_markers[1]

                u1, v1 = tag1["center"]
                u2, v2 = tag2["center"]

                mid_u = int((u1 + u2) / 2)
                mid_v = int((v1 + v2) / 2)

                mid_z = (tag1["distance_m"] + tag2["distance_m"]) / 2
                mid_x, mid_y, mid_z = self.pixel_to_camera_direction(mid_u, mid_v, mid_z)

                angle_x_rad = math.atan2(mid_x, mid_z)
                angle_x_deg = math.degrees(angle_x_rad)
                
                # Aruco gorunur oldu
                visible_msg.data = True
                
                # Aruco bilgilerini yayinla
                # x -> Acisal sapma (angle_x_deg)
                # y -> Kameraya olan X mesafesi (mid_x) - yatay öteleme
                # z -> Ileriye olan Z mesafesi (mid_z)
                midpoint_msg = Point()
                midpoint_msg.x = float(angle_x_deg)
                midpoint_msg.y = float(mid_x) 
                midpoint_msg.z = float(mid_z)
                self.midpoint_pub.publish(midpoint_msg)

        # Gorunurluk bilgisini yayinla
        self.visible_pub.publish(visible_msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
