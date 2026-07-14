#!/usr/bin/env python3
import subprocess
import time
import cv2
import numpy as np
import math
import sys

# =========================
# SETTINGS
# =========================
TAG_SIZE_M = 0.20  # 20x20 cm ArUco tag
HFOV_DEG = 62.2   # Raspberry Pi Camera V2 (Sony IMX219 standard) horizontal FOV


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


class ArucoDetectorStandalone:
    def __init__(self):
        try:
            self.cap = RPI_Camera()
        except Exception as e:
            print(f"Failed to start camera: {e}")
            sys.exit(1)

        # Görüntü gelene kadar birkaç kez okumayı dene (Kamera ısınma süresi)
        ret = False
        for _ in range(20):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.1)
            
        if not ret:
            print("Camera opened but failed to get the first frame.")
            self.cap.release()
            sys.exit(1)
        
        print(f"Camera is running. Size: {frame.shape}")
        self.IMAGE_H, self.IMAGE_W = frame.shape[:2]
        
        self.fx = (self.IMAGE_W / 2) / math.tan(math.radians(HFOV_DEG / 2))
        self.fy = self.fx
        self.cx = self.IMAGE_W / 2
        self.cy = self.IMAGE_H / 2
        
        # ArUco settings
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        else:
            self.use_new_api = False
            
        print("ArUco detector initialized. Starting loop...")

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

    def run(self):
        try:
            while True:
                start_time = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    print("Could not get camera frame.")
                    time.sleep(0.1)
                    continue

                # İlk kareyi kameranın ne gördüğünü doğrulamak için diske kaydet
                if not self.debug_frame_saved:
                    cv2.imwrite("debug_camera.jpg", frame)
                    print("!!! [DEBUG] Kameranın gördüğü ilk kare 'debug_camera.jpg' olarak kaydedildi.")
                    self.debug_frame_saved = True

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if self.use_new_api:
                    corners, ids, rejected = self.detector.detectMarkers(gray)
                else:
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        gray,
                        self.aruco_dict,
                        parameters=self.aruco_params
                    )

                if ids is not None and len(ids) >= 1:
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

                    if len(detected_markers) >= 1:
                        tag = detected_markers[0]
                        u, v = tag["center"]
                        z = tag["distance_m"]
                        x, y, z = self.pixel_to_camera_direction(u, v, z)

                        angle_x_rad = math.atan2(x, z)
                        angle_x_deg = math.degrees(angle_x_rad)

                        print(f"[FOUND] ID: {tag['id']} | Distance: {z:.2f}m | Angle: {angle_x_deg:.1f}deg")
                    else:
                        print("[NOT FOUND] No tags passed validation.")
                else:
                    print("[NOT FOUND] No tags detected.")

                # Target ~15 FPS
                elapsed = time.time() - start_time
                sleep_time = max(0.0, (1.0 / 15.0) - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nShutting down detector...")
        finally:
            self.cap.release()
            print("Camera released. Exiting.")


if __name__ == '__main__':
    detector = ArucoDetectorStandalone()
    detector.run()
