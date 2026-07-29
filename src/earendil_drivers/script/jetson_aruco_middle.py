# Bu script Jetson Nano üzerinde çalışmaktadır ve bir systemd servisi (aruco_perception.service) olarak arka planda otomatik başlatılır.
# Raspberry Pi 5 ile ethernet kablosu üzerinden kurulan yerel ağ (LAN) bağlantısı üzerinden haberleşir.
# Kamera görüntüsünü işleyerek ArUco tag tespitlerini yapar ve elde edilen verileri UDP soketleri aracılığıyla Raspberry Pi 5'e iletir.
import cv2
import numpy as np
import math
import socket
import json
import select
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


# =========================
# AYARLAR
# =========================

TAG_SIZE_M = 0.20  # 20x20 cm ArUco tag
HFOV_DEG = 83.0   # IMX219-83 yaklaşık yatay görüş açısı

# =========================
# NETWORKING & DISPLAY SETTINGS
# =========================
UDP_IP = "192.168.1.100"       # Target Raspberry Pi 5 IP
UDP_PORT_SEND = 5005           # Port to stream ArUco data
UDP_PORT_RECV = 5006           # Port to listen for START/STOP commands
SHOW_DISPLAY = True            # Enable/Disable GUI window (set to False for headless run)



# =========================
# CSI KAMERA SINIFI
# =========================

class CSI_Camera:
    def __init__(self):
        Gst.init(None)

        self.pipeline_str = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! "
            "nvvidconv ! "
            "video/x-raw, width=1280, height=720, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink max-buffers=1 drop=true sync=false"
        )

        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")

        self.pipeline.set_state(Gst.State.PLAYING)

    def read(self):
        sample = self.appsink.emit("try-pull-sample", Gst.SECOND)

        if sample is None:
            return False, None

        buffer = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)

        width = structure.get_value("width")
        height = structure.get_value("height")

        success, map_info = buffer.map(Gst.MapFlags.READ)

        if not success:
            return False, None

        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8)
            frame = frame.reshape((height, width, 3))
            frame = frame.copy()
        finally:
            buffer.unmap(map_info)

        return True, frame

    def release(self):
        self.pipeline.set_state(Gst.State.NULL)


# =========================
# KAMERA PARAMETRELERİ (Static 1280x720)
# =========================
IMAGE_W = 1280
IMAGE_H = 720

fx = (IMAGE_W / 2) / math.tan(math.radians(HFOV_DEG / 2))
fy = fx
cx = IMAGE_W / 2
cy = IMAGE_H / 2

camera_matrix = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32)

print("Kamera parametreleri:")
print("IMAGE_W:", IMAGE_W)
print("IMAGE_H:", IMAGE_H)
print("fx:", fx)
print("fy:", fy)
print("cx:", cx)
print("cy:", cy)

# =========================
# NETWORKING BAŞLANGICI
# =========================
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_recv.bind(('0.0.0.0', UDP_PORT_RECV))
sock_recv.setblocking(False)

print(f"Network dinleyicisi baslatildi. Port: {UDP_PORT_RECV}")
print(f"Veri gonderilecek adres: {UDP_IP}:{UDP_PORT_SEND}")



# =========================
# ARUCO AYARLARI
# =========================

if hasattr(cv2.aruco, "getPredefinedDictionary"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
else:
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

if hasattr(cv2.aruco, "DetectorParameters"):
    aruco_params = cv2.aruco.DetectorParameters()
else:
    aruco_params = cv2.aruco.DetectorParameters_create()


# =========================
# FONKSİYONLAR
# =========================

def marker_center(corners):
    c = np.mean(corners, axis=0)
    return int(c[0]), int(c[1])


def marker_area(corners):
    return cv2.contourArea(corners.astype(np.float32))


def distance_from_area(area_px):
    if area_px <= 0:
        return None

    side_px = math.sqrt(area_px)
    z_m = fx * TAG_SIZE_M / side_px
    return z_m


def pixel_to_camera_direction(u, v, z):
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z


# =========================
# ANA DÖNGÜ (State Machine)
# =========================

while True:
    print("\n[IDLE] Raspberry Pi 5'ten START komutu bekleniyor...")
    active = False
    pi_ip = None
    
    # Outer Loop: START paketini bekle
    while not active:
        r, _, _ = select.select([sock_recv], [], [], 1.0)
        if r:
            try:
                data, addr = sock_recv.recvfrom(1024)
                cmd = data.decode('utf-8').strip()
                if cmd == "START":
                    pi_ip = addr[0]
                    print(f"[IDLE] START komutu alindi ({addr}). Kamera aciliyor...")
                    active = True
            except Exception as e:
                print(f"[IDLE] Komut okuma hatasi: {e}")
                time.sleep(0.1)

    # Kamera baslatma
    cap = None
    try:
        cap = CSI_Camera()
        time.sleep(0.5)  # Kameranin kendine gelmesi icin kisa bekleme
    except Exception as e:
        print(f"[HATA] Kamera baslatilamadi: {e}")
        continue

    last_heartbeat = time.time()

    # Inner Loop: Frame isleme ve veri gonderme
    while active:
        # Gelen komutlari kontrol et (START = heartbeat, STOP = durdur)
        try:
            while True:
                r, _, _ = select.select([sock_recv], [], [], 0.0)
                if r:
                    data, addr = sock_recv.recvfrom(1024)
                    cmd = data.decode('utf-8').strip()
                    if cmd == "STOP":
                        print("[RUNNING] STOP komutu alindi. Kamera kapatiliyor...")
                        active = False
                        break
                    elif cmd == "START":
                        last_heartbeat = time.time()
                else:
                    break
        except Exception as e:
            print(f"[RUNNING] Komut kontrol hatasi: {e}")

        if not active:
            break

        # Heartbeat timeout kontrolü (2.5 saniye Pi5'ten haber gelmezse kapat)
        if time.time() - last_heartbeat > 2.5:
            print("[RUNNING] Heartbeat zaman asimi! Raspberry baglantisi koptu. Kamera kapatiliyor...")
            active = False
            break

        # Görüntü yakalama
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[RUNNING] Kamera goruntusu alinamadi.")
            time.sleep(0.03)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            aruco_dict,
            parameters=aruco_params
        )

        visible = False
        angle_val = 0.0
        distance_val = 0.0
        x_offset_val = 0.0

        if ids is not None and len(ids) >= 2:
            ids = ids.flatten()
            detected_markers = []

            for i, marker_id in enumerate(ids):
                marker_corners = corners[i][0]
                area_px = marker_area(marker_corners)
                z_m = distance_from_area(area_px)

                if z_m is None:
                    continue

                u, v = marker_center(marker_corners)

                detected_markers.append({
                    "id": int(marker_id),
                    "center": (u, v),
                    "area_px": area_px,
                    "distance_m": z_m,
                    "corners": marker_corners
                })

                if SHOW_DISPLAY:
                    cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"ID:{marker_id} Z:{z_m:.2f}m",
                        (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

            if len(detected_markers) >= 2:
                detected_markers.sort(key=lambda m: m["center"][0])
                tag1 = detected_markers[0]
                tag2 = detected_markers[1]

                u1, v1 = tag1["center"]
                u2, v2 = tag2["center"]

                mid_u = int((u1 + u2) / 2)
                mid_v = int((v1 + v2) / 2)
                mid_z = (tag1["distance_m"] + tag2["distance_m"]) / 2

                mid_x, mid_y, mid_z = pixel_to_camera_direction(mid_u, mid_v, mid_z)
                angle_x_rad = math.atan2(mid_x, mid_z)
                angle_x_deg = math.degrees(angle_x_rad)

                visible = True
                angle_val = angle_x_deg
                distance_val = mid_z
                x_offset_val = mid_x

                print(
                    f"TAG1 ID:{tag1['id']} Z:{tag1['distance_m']:.2f} m | "
                    f"TAG2 ID:{tag2['id']} Z:{tag2['distance_m']:.2f} m | "
                    f"MID: X={mid_x:.2f}m Z={mid_z:.2f}m Angle={angle_x_deg:.2f}deg"
                )

                if SHOW_DISPLAY:
                    cv2.circle(frame, (mid_u, mid_v), 10, (255, 0, 0), -1)
                    cv2.line(frame, (u1, v1), (u2, v2), (255, 0, 0), 2)
                    cv2.putText(
                        frame,
                        f"MID Z:{mid_z:.2f}m Angle:{angle_x_deg:.1f}deg",
                        (mid_u + 15, mid_v + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 0),
                        2
                    )

        # UDP paketi gönder
        data_to_send = {
            "visible": visible,
            "angle": angle_val,
            "distance": distance_val,
            "x_offset": x_offset_val
        }
        try:
            payload = json.dumps(data_to_send).encode('utf-8')
            sock_send.sendto(payload, (pi_ip, UDP_PORT_SEND))
        except Exception as e:
            pass

        # Ekran penceresi gösterme (Başsız/Headless modda çalışırken hata vermemesi için)
        if SHOW_DISPLAY:
            try:
                cv2.imshow("Aruco Midpoint Distance", frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    print("ESC basildi, cikiliyor...")
                    active = False
                    break
            except Exception as e:
                print(f"Gorsel sunucu bulunamadi (headless run?): {e}")
                SHOW_DISPLAY = False
                try:
                    cv2.destroyAllWindows()
                except:
                    pass

    # Kamerayı bırak
    if cap:
        try:
            cap.release()
            print("Kamera birakildi.")
        except Exception as e:
            print(f"Kamera birakma hatasi: {e}")

    if SHOW_DISPLAY:
        try:
            cv2.destroyAllWindows()
        except:
            pass