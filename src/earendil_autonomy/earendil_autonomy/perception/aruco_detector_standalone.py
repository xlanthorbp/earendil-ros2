#!/usr/bin/env python3
# DIKKAT: Bu klasördeki (earendil_bot/scripts/) kodlar sadece örnek/test kodlarıdır.
# Raspberry Pi 5 + Raspberry Pi Camera V2 (Sony IMX219) + Raspberry Pi OS (Bookworm) / Ubuntu uyumlu ArUco Test Scripti.
import subprocess
import time
import cv2
import numpy as np
import math
import sys
import shutil

# =========================
# SETTINGS
# =========================
TAG_SIZE_M = 0.20  # 20x20 cm ArUco tag
HFOV_DEG = 62.2   # Raspberry Pi Camera V2 (Sony IMX219 standard) horizontal FOV


class RPI_Camera:
    """
    Raspberry Pi 5 + IMX219 (Camera V2) kamera için rpicam-vid / libcamera-vid veya OpenCV V4L2 tabanlı görüntü yakalama.
    Ubuntu 24.04 / Raspberry Pi OS üzerinde tam uyumluluk sağlar.
    """
    def __init__(self, width=1280, height=720, framerate=30):
        self.width = width
        self.height = height
        self.use_v4l2 = False
        
        # Bookworm / Ubuntu 24.04 üzerinde komut adını belirle (rpicam-vid veya libcamera-vid)
        cam_cmd = 'rpicam-vid' if shutil.which('rpicam-vid') else ('libcamera-vid' if shutil.which('libcamera-vid') else None)
        
        if cam_cmd:
            self.frame_size = width * height * 3 // 2
            cmd = [
                cam_cmd,
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
            time.sleep(0.5)
        else:
            print("Bilgi: 'rpicam-vid' veya 'libcamera-vid' bulunamadı. OpenCV V4L2 (cv2.VideoCapture) moduna geçiliyor...")
            self.use_v4l2 = True
            self.v4l2_cap = cv2.VideoCapture(0)
            self.v4l2_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.v4l2_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.v4l2_cap.set(cv2.CAP_PROP_FPS, framerate)
            if not self.v4l2_cap.isOpened():
                raise RuntimeError("Ne libcamera CLI araçları (rpicam-vid/libcamera-vid) ne de OpenCV V4L2 (/dev/video0) kamerası açılabildi.")

    def read(self):
        if self.use_v4l2:
            return self.v4l2_cap.read()
        else:
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
        if self.use_v4l2:
            if hasattr(self, 'v4l2_cap') and self.v4l2_cap:
                self.v4l2_cap.release()
        else:
            if hasattr(self, 'process') and self.process:
                self.process.terminate()
                self.process.wait()


class ArucoDetectorStandalone:
    def __init__(self, width=1280, height=720, framerate=30, show_display=True):
        self.debug_frame_saved = False
        self.show_display = show_display
        self.framerate = framerate
        try:
            self.cap = RPI_Camera(width=width, height=height, framerate=framerate)
        except Exception as e:
            print(f"Kamera başlatılamadı: {e}")
            sys.exit(1)

        # Görüntü gelene kadar birkaç kez okumayı dene (Kamera ısınma süresi)
        ret = False
        for _ in range(20):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.1)
            
        if not ret:
            print("Kamera açıldı ancak ilk kare alınamadı.")
            self.cap.release()
            sys.exit(1)
        
        print(f"Kamera çalışıyor. Çözünürlük: {frame.shape}")
        self.IMAGE_H, self.IMAGE_W = frame.shape[:2]
        
        self.fx = (self.IMAGE_W / 2) / math.tan(math.radians(HFOV_DEG / 2))
        self.fy = self.fx
        self.cx = self.IMAGE_W / 2
        self.cy = self.IMAGE_H / 2
        
        # ArUco ayarları - OpenCV sürüm uyumluluğu (OpenCV 4.x / Bookworm python3-opencv)
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        else:
            self.use_new_api = False
            
        print("ArUco detektörü başlatıldı. Döngü başlatılıyor...")
        if self.show_display:
            print("Pencere modu AKTİF. Çıkmak için pencere açıkken ESC tuşuna basın.")

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
                    print("Kamera karesi alınamadı.")
                    time.sleep(0.1)
                    continue

                # İlk kareyi kameranın ne gördüğünü doğrulamak için diske kaydet
                if not self.debug_frame_saved:
                    cv2.imwrite("debug_camera.jpg", frame)
                    print("!!! [DEBUG] Kameranın gördüğü ilk kare 'debug_camera.jpg' olarak kaydedildi.")
                    self.debug_frame_saved = True

                # Görsel test için kamera merkezine mavi hedef artı işareti (crosshair) çiz
                if self.show_display:
                    icx, icy = int(self.cx), int(self.cy)
                    cv2.line(frame, (icx - 15, icy), (icx + 15, icy), (255, 0, 0), 1)
                    cv2.line(frame, (icx, icy - 15), (icx, icy + 15), (255, 0, 0), 1)

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
                            "distance_m": z_m,
                            "corners": marker_corners
                        })

                        # Ekranda ArUco çerçevesini yeşil, merkezini kırmızı çiz
                        if self.show_display:
                            cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 255, 0), 2)
                            cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

                    if len(detected_markers) >= 1:
                        tag = detected_markers[0]
                        u, v = tag["center"]
                        z = tag["distance_m"]
                        x, y, z = self.pixel_to_camera_direction(u, v, z)

                        angle_x_rad = math.atan2(x, z)
                        angle_x_deg = math.degrees(angle_x_rad)

                        print(f"[FOUND] ID: {tag['id']} | X: {x:+.3f}m | Y: {y:+.3f}m | Z (Dist): {z:.2f}m | Angle: {angle_x_deg:+.1f}deg")

                        # Ekran üzerine bilgileri yazdır
                        if self.show_display:
                            info_text1 = f"ID:{tag['id']}  Z:{z:.2f}m  Angle:{angle_x_deg:+.1f}deg"
                            info_text2 = f"X:{x:+.2f}m  Y:{y:+.2f}m"
                            cv2.putText(frame, info_text1, (u + 10, v - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(frame, info_text2, (u + 10, v - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    else:
                        print("[NOT FOUND] No tags passed validation.")
                else:
                    print("[NOT FOUND] No tags detected.")

                # Ekranda pencereyi göster
                if self.show_display:
                    try:
                        cv2.imshow("ArUco Detector Test", frame)
                        key = cv2.waitKey(1)
                        if key == 27:  # ESC tuşu
                            print("ESC basıldı, çıkılıyor...")
                            break
                    except Exception as e:
                        print(f"Görsel ekran sunucusu bulunamadı (headless mod?): {e}")
                        self.show_display = False

                # Hedef FPS bekleme süresi
                elapsed = time.time() - start_time
                sleep_time = max(0.0, (1.0 / self.framerate) - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nDetektör kapatılıyor...")
        finally:
            self.cap.release()
            if self.show_display:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            print("Kamera serbest bırakıldı. Çıkış yapılıyor.")


if __name__ == '__main__':
    detector = ArucoDetectorStandalone()
    detector.run()

