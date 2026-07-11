#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import subprocess
import time

import cv2
import numpy as np
import math

# =========================
# SETTINGS
# =========================
TAG_SIZE_M = 0.20  # 20x20 cm ArUco tag
HFOV_DEG = 83.0   # IMX219-83 approximate horizontal FOV


class RPI_Camera:
    """
    Raspberry Pi 5 + IMX219 kamera için rpicam-vid tabanlı görüntü yakalama.
    Ubuntu 24.04'teki libcamera (v0.2.0) PiSP desteklemediği için,
    Pi Foundation'ın kendi rpicam-vid aracını subprocess olarak kullanıyoruz.
    """
    def __init__(self, width=1280, height=720, framerate=15):
        self.width = width
        self.height = height
        # YUV420 frame boyutu: width * height * 1.5
        self.frame_size = width * height * 3 // 2
        
        cmd = [
            'rpicam-vid',
            '-t', '0',              # Süresiz çalış
            '--width', str(width),
            '--height', str(height),
            '--framerate', str(framerate),
            '--codec', 'yuv420',     # Ham YUV420 formatı
            '-n',                    # Önizleme penceresi açma
            '-o', '-'                # stdout'a yaz
        ]
        
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self.frame_size * 2
        )
        
        # Kameranın ısınması için ilk birkaç kareyi bekle
        time.sleep(0.5)

    def read(self):
        try:
            raw = self.process.stdout.read(self.frame_size)
            if len(raw) != self.frame_size:
                return False, None
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            return True, frame
        except Exception:
            return False, None

    def release(self):
        if self.process:
            self.process.terminate()
            self.process.wait()


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.midpoint_pub = self.create_publisher(Point, '/aruco_midpoint', 10)
        self.visible_pub = self.create_publisher(Bool, '/aruco_visible', 10)
        
        try:
            self.cap = RPI_Camera()
        except Exception as e:
            self.get_logger().error(f"Failed to start camera: {e}")
            self.cap = None

        if self.cap:
            # Görüntü gelene kadar birkaç kez okumayı dene (Kamera ısınma süresi)
            ret = False
            for _ in range(20):
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    break
                import time
                time.sleep(0.1)
                
            if not ret:
                self.get_logger().error("Camera opened but failed to get the first frame.")
            else:
                self.get_logger().info(f"Camera is running. Size: {frame.shape}")
                self.IMAGE_H, self.IMAGE_W = frame.shape[:2]
                
                self.fx = (self.IMAGE_W / 2) / math.tan(math.radians(HFOV_DEG / 2))
                self.fy = self.fx
                self.cx = self.IMAGE_W / 2
                self.cy = self.IMAGE_H / 2
                
                # ArUco settings - Eski API'yi kullan (OpenCV < 4.7)
                # Segfault'u önlemek için DetectorParameters() yerine DetectorParameters_create() kullanıyoruz
                try:
                    self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
                    self.aruco_params = cv2.aruco.DetectorParameters_create()
                except AttributeError:
                    # Alternatif fallback (olur da farklı bir versiyonsa)
                    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
                    self.aruco_params = cv2.aruco.DetectorParameters()
                    
                self.get_logger().info("ArUco detector initialized.")
                    
                # Timer - call 15 times a second (15 Hz)
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
            self.get_logger().warn("Could not get camera frame.", throttle_duration_sec=2.0)
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
                # Sort from left to right
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
                
                # ArUco became visible
                visible_msg.data = True
                
                # Publish ArUco information
                # x -> Angular deviation (angle_x_deg)
                # y -> X distance to camera (mid_x) - horizontal translation
                # z -> Z distance forward (mid_z)
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
