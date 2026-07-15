#!/usr/bin/env python3
import cv2
import numpy as np
import math
import socket
import json
import select
import time
from pathlib import Path

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

# =========================
# AYARLAR
# =========================
ROCK_SIZE_M = 0.15   # 15 cm default rock size
HFOV_DEG = 83.0      # IMX219-83 yaklaşık yatay görüş açısı

# =========================
# NETWORKING & DISPLAY SETTINGS
# =========================
UDP_IP = "192.168.1.100"       # Target Raspberry Pi 5 IP
UDP_PORT_SEND = 5007           # Port to stream rock data
UDP_PORT_RECV = 5008           # Port to listen for START/STOP commands
SHOW_DISPLAY = True            # Enable/Disable GUI window (set to False for headless run)

# Algılama Parametreleri
CHROMA_WEIGHT = 2.5
MIN_CENTER_Y_RATIO = 0.15
BOX_MARGIN_RATIO = 0.10

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


# =========================
# DETECTOR FUNCTIONS
# =========================
def detect_horizon_y(lightness, row_threshold=35, padding=5):
    height = lightness.shape[0]
    row_medians = np.median(lightness, axis=1)

    y = 0
    while y < height and row_medians[y] < row_threshold:
        y += 1

    if y >= 5:
        return min(height - 1, y + padding)
    return 0


def build_foreground_mask(image, cluster_count=5):
    height, width = image.shape[:2]
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lightness = lab[:, :, 0]

    roi_start_y = detect_horizon_y(lightness)
    roi = lab[roi_start_y:].astype(np.float32)
    pixels = roi.reshape(-1, 3)

    if len(pixels) == 0:
        return np.zeros((height, width), dtype=np.uint8), roi_start_y

    max_samples = 30000
    if len(pixels) > max_samples:
        rng = np.random.default_rng(0)
        indices = rng.choice(len(pixels), max_samples, replace=False)
        samples = np.ascontiguousarray(pixels[indices], dtype=np.float32)
    else:
        samples = np.ascontiguousarray(pixels, dtype=np.float32)

    actual_cluster_count = min(cluster_count, len(samples))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)

    _, labels, centers = cv2.kmeans(
        samples,
        actual_cluster_count,
        None,
        criteria,
        5,
        cv2.KMEANS_PP_CENTERS,
    )

    cluster_sizes = np.bincount(labels.ravel(), minlength=actual_cluster_count)
    background_center = centers[int(np.argmax(cluster_sizes))]

    color_distance = np.sqrt(np.sum((roi - background_center) ** 2, axis=2))
    distance_8bit = cv2.normalize(color_distance, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    _, roi_mask = cv2.threshold(distance_8bit, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, np.ones((11, 11), dtype=np.uint8))

    full_mask = np.zeros((height, width), dtype=np.uint8)
    full_mask[roi_start_y:] = roi_mask
    return full_mask, roi_start_y


def split_regions_with_watershed(image, foreground_mask):
    height, width = foreground_mask.shape
    opened = cv2.morphologyEx(foreground_mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    distance = cv2.distanceTransform(opened, cv2.DIST_L2, 5)

    markers = np.zeros((height, width), dtype=np.int32)
    next_label = 1

    component_count, components, stats, _ = cv2.connectedComponentsWithStats(opened)

    for component_id in range(1, component_count):
        _x, _y, _w, _h, area = stats[component_id]
        if area < 0.0004 * height * width:
            continue

        component_mask = components == component_id
        local_distance = distance * component_mask.astype(np.float32)
        maximum_distance = float(local_distance.max())

        if maximum_distance < 2:
            continue

        sure_foreground = (local_distance > 0.35 * maximum_distance).astype(np.uint8) * 255
        sure_foreground = cv2.morphologyEx(sure_foreground, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))

        seed_count, seeds = cv2.connectedComponents(sure_foreground)
        for seed_id in range(1, seed_count):
            seed_mask = seeds == seed_id
            if np.count_nonzero(seed_mask) < 50:
                continue
            markers[seed_mask] = next_label
            next_label += 1

    if next_label == 1:
        return []

    background_label = next_label
    markers[foreground_mask == 0] = background_label
    watershed_result = cv2.watershed(image.copy(), markers)

    regions = []
    for label in range(1, background_label):
        region_mask = (watershed_result == label).astype(np.uint8) * 255
        area = cv2.countNonZero(region_mask)
        if area == 0:
            continue

        contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        contour = max(contours, key=cv2.contourArea)
        regions.append({
            "mask": region_mask,
            "box": cv2.boundingRect(contour),
            "area": area
        })

    return regions


def is_valid_region(region, image_shape, roi_start_y):
    height, width = image_shape[:2]
    x, y, box_width, box_height = region["box"]
    area = region["area"]

    if area < 0.0015 * height * width:
        return False
    if box_width < 0.03 * width or box_height < 0.07 * height:
        return False
    if box_width > 0.45 * width or box_height > 0.65 * height:
        return False
    if box_width / max(box_height, 1) > 4.0:
        return False
    if box_height / max(box_width, 1) > 3.0:
        return False
    if x <= 1 or x + box_width >= width - 1:
        return False

    center_y = y + box_height / 2
    minimum_center_y = roi_start_y + MIN_CENTER_Y_RATIO * (height - roi_start_y)
    return center_y >= minimum_center_y


def grabcut_candidate(image, box):
    image_height, image_width = image.shape[:2]
    x, y, box_width, box_height = box

    context_x = int(0.15 * box_width)
    context_y = int(0.18 * box_height)

    crop_x1 = max(0, x - context_x)
    crop_y1 = max(0, y - context_y)
    crop_x2 = min(image_width, x + box_width + context_x)
    crop_y2 = min(image_height, y + box_height + context_y)

    crop = image[crop_y1:crop_y2, crop_x1:crop_x2].copy()
    crop_height, crop_width = crop.shape[:2]

    inner_x = max(1, x - crop_x1)
    inner_y = max(1, y - crop_y1)
    inner_width = min(box_width, crop_width - inner_x - 2)
    inner_height = min(box_height, crop_height - inner_y - 2)

    if inner_width < 2 or inner_height < 2:
        return None

    grabcut_mask = np.zeros((crop_height, crop_width), dtype=np.uint8)
    background_model = np.zeros((1, 65), dtype=np.float64)
    foreground_model = np.zeros((1, 65), dtype=np.float64)

    try:
        cv2.grabCut(
            crop,
            grabcut_mask,
            (inner_x, inner_y, inner_width, inner_height),
            background_model,
            foreground_model,
            4,
            cv2.GC_INIT_WITH_RECT,
        )
    except cv2.error:
        return None

    foreground = np.where(
        (grabcut_mask == cv2.GC_FGD) | (grabcut_mask == cv2.GC_PR_FGD),
        255,
        0,
    ).astype(np.uint8)

    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(foreground)
    if component_count <= 1:
        return None

    expected_center = np.array([inner_x + inner_width / 2, inner_y + inner_height / 2], dtype=float)
    best_component_id = None
    best_component_score = -1.0

    for component_id in range(1, component_count):
        area = stats[component_id, cv2.CC_STAT_AREA]
        if area < 100:
            continue

        center_distance = np.linalg.norm(centroids[component_id] - expected_center)
        score = area / (1 + 0.02 * center_distance)

        if score > best_component_score:
            best_component_score = score
            best_component_id = component_id

    if best_component_id is None:
        return None

    local_mask = (labels == best_component_id).astype(np.uint8) * 255
    global_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    global_mask[crop_y1:crop_y2, crop_x1:crop_x2] = local_mask
    return global_mask


def evaluate_candidate(image, region):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    foreground_mask = grabcut_candidate(image, region["box"])

    if foreground_mask is None:
        foreground_mask = region["mask"].copy()

    y_coordinates, x_coordinates = np.where(foreground_mask > 0)
    if len(x_coordinates) < 100:
        return None

    minimum_y = int(y_coordinates.min())
    maximum_y = int(y_coordinates.max())
    evaluation_mask = foreground_mask.copy()

    shadow_cut_y = int(minimum_y + 0.80 * (maximum_y - minimum_y + 1))
    evaluation_mask[shadow_cut_y:maximum_y + 1, :] = 0

    eroded_mask = cv2.erode(evaluation_mask, np.ones((5, 5), dtype=np.uint8))
    if cv2.countNonZero(eroded_mask) > 100:
        evaluation_mask = eroded_mask

    selected_lightness = lab[:, :, 0][evaluation_mask > 0]
    if len(selected_lightness) < 100:
        return None

    percentile_20, percentile_80 = np.percentile(selected_lightness, [20, 80])
    trimmed_pixels = selected_lightness[(selected_lightness >= percentile_20) & (selected_lightness <= percentile_80)]

    brightness = float(np.mean(trimmed_pixels) if len(trimmed_pixels) else np.mean(selected_lightness))

    a_channel = lab[:, :, 1].astype(np.float32) - 128.0
    b_channel = lab[:, :, 2].astype(np.float32) - 128.0
    chroma_map = np.sqrt(a_channel**2 + b_channel**2)
    chroma = float(np.median(chroma_map[evaluation_mask > 0]))

    return {
        "score": brightness + CHROMA_WEIGHT * chroma,
        "brightness": brightness,
        "chroma": chroma,
        "mask": foreground_mask,
        "region": region,
    }


def object_box_from_mask(mask, fallback_box):
    y_coordinates, x_coordinates = np.where(mask > 0)
    if len(x_coordinates) < 100:
        return fallback_box

    minimum_y = int(y_coordinates.min())
    maximum_y = int(y_coordinates.max())
    object_height = maximum_y - minimum_y + 1

    row_indices = np.indices(mask.shape)[0]
    upper_part = (mask > 0) & (row_indices < minimum_y + 0.62 * object_height)

    upper_y, upper_x = np.where(upper_part)
    if len(upper_x) < 100:
        return fallback_box

    minimum_x = int(upper_x.min())
    maximum_x = int(upper_x.max())

    horizontal_margin = max(2, int(0.04 * (maximum_x - minimum_x + 1)))
    minimum_x = max(0, minimum_x - horizontal_margin)
    maximum_x = min(mask.shape[1] - 1, maximum_x + horizontal_margin)

    column_indices = np.indices(mask.shape)[1]
    restricted_mask = (mask > 0) & (column_indices >= minimum_x) & (column_indices <= maximum_x)

    restricted_y, restricted_x = np.where(restricted_mask)
    if len(restricted_x) < 100:
        return fallback_box

    minimum_y = int(restricted_y.min())
    maximum_y = int(restricted_y.max())

    return (minimum_x, minimum_y, maximum_x - minimum_x + 1, maximum_y - minimum_y + 1)


def make_square_box(box, image_shape):
    x, y, box_width, box_height = box
    image_height, image_width = image_shape[:2]

    center_x = x + box_width // 2
    center_y = y + box_height // 2
    square_size = max(2, int(math.ceil(max(box_width, box_height) * (1 + BOX_MARGIN_RATIO))))

    half_size = square_size // 2
    x1 = max(0, center_x - half_size)
    y1 = max(0, center_y - half_size)
    x2 = min(image_width - 1, x1 + square_size - 1)
    y2 = min(image_height - 1, y1 + square_size - 1)

    x1 = max(0, x2 - square_size + 1)
    y1 = max(0, y2 - square_size + 1)

    return x1, y1, x2, y2, center_x, center_y


def detect_darkest_stone(image):
    foreground_mask, roi_start_y = build_foreground_mask(image)
    regions = split_regions_with_watershed(image, foreground_mask)

    regions = [r for r in regions if is_valid_region(r, image.shape, roi_start_y)]
    if not regions:
        return None, foreground_mask

    largest_area = max(r["area"] for r in regions)
    regions = [r for r in regions if r["area"] >= 0.12 * largest_area]

    candidates = []
    for region in regions:
        candidate = evaluate_candidate(image, region)
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        return None, foreground_mask

    darkest_candidate = min(candidates, key=lambda item: item["score"])
    object_box = object_box_from_mask(darkest_candidate["mask"], darkest_candidate["region"]["box"])
    darkest_candidate["object_box"] = object_box
    darkest_candidate["square_box"] = make_square_box(object_box, image.shape)
    return darkest_candidate, foreground_mask


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

        result, debug_mask = detect_darkest_stone(frame)

        visible = False
        angle_val = 0.0
        distance_val = 0.0
        x_offset_val = 0.0
        score_val = 0.0

        if result is not None:
            x1, y1, x2, y2, center_x, center_y = result["square_box"]
            box_width = x2 - x1 + 1

            # Açısal sapma hesabı
            angle_val = math.degrees(math.atan2(center_x - cx, fx))
            
            # Normalleştirilmiş yatay sapma (-1.0 ile 1.0 arası)
            x_offset_val = float((center_x - cx) / cx)
            
            # Kutu genişliğine göre mesafe tahmini
            if box_width > 0:
                distance_val = float(fx * ROCK_SIZE_M / box_width)
            else:
                distance_val = 0.0

            score_val = float(result["score"])
            visible = True

            print(
                f"[OK] Rapor: visible={visible} | Açı={angle_val:.1f}° | "
                f"Mesafe={distance_val:.2f}m | Offset={x_offset_val:.2f} | Skor={score_val:.2f}"
            )

            if SHOW_DISPLAY:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
                cv2.circle(frame, (center_x, center_y), 8, (0, 0, 255), -1)
                label = f"Rock: ({center_x}, {center_y}) Dist: {distance_val:.1f}m"
                cv2.putText(
                    frame,
                    label,
                    (x1, max(35, y1 - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # UDP paketi gönder
        data_to_send = {
            "visible": visible,
            "angle": angle_val,
            "distance": distance_val,
            "x_offset": x_offset_val,
            "score": score_val
        }
        try:
            payload = json.dumps(data_to_send).encode('utf-8')
            sock_send.sendto(payload, (pi_ip, UDP_PORT_SEND))
        except Exception as e:
            pass

        # Ekran penceresi gösterme (Başsız/Headless modda çalışırken hata vermemesi için)
        if SHOW_DISPLAY:
            try:
                cv2.imshow("Rock Detector Bridge", frame)
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
