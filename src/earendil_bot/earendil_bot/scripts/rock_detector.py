from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import cv2
import numpy as np


# ============================================================
# AYARLAR
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
INPUT_DIR = SCRIPT_DIR / "input_images"
OUTPUT_DIR = SCRIPT_DIR / "output_images"
DEBUG_DIR = OUTPUT_DIR / "debug_masks"

SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}

# 0.0 = yalnızca koyuluk ölçülür.
# 2.0 = koyu mavi/kahverengi yerine siyaha-griye yakın koyu taş tercih edilir.
CHROMA_WEIGHT = 2.5

# Taşın merkezi, ufuktan sonraki kullanılabilir yüksekliğin en az bu oranında aşağıda olsun.
# Uzak taşları da almak için 0.15'e düşürülebilir.
MIN_CENTER_Y_RATIO = 0.15

# Taşın etrafındaki kare kutunun payı.
BOX_MARGIN_RATIO = 0.10

SAVE_DEBUG_MASKS = True


# ============================================================
# WINDOWS'TA TÜRKÇE KARAKTERLİ YOLLAR İÇİN GÖRSEL OKUMA/YAZMA
# ============================================================


def unicode_imread(file_path: Path | str) -> np.ndarray | None:
    """Türkçe karakter içeren Windows yollarından görüntü okur."""
    try:
        data = np.fromfile(str(file_path), dtype=np.uint8)
        if data.size == 0:
            return None
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except (OSError, ValueError, cv2.error) as error:
        print(f"[HATA] Görüntü okuma hatası: {file_path}\n       {error}")
        return None


def unicode_imwrite(file_path: Path | str, image: np.ndarray) -> bool:
    """Türkçe karakter içeren Windows yollarına görüntü kaydeder."""
    path = Path(file_path)
    extension = path.suffix.lower() or ".png"

    try:
        success, encoded = cv2.imencode(extension, image)
        if not success:
            return False
        encoded.tofile(str(path))
        return True
    except (OSError, ValueError, cv2.error) as error:
        print(f"[HATA] Görüntü kaydetme hatası: {path}\n       {error}")
        return False


# ============================================================
# ÖN PLAN / TAŞ ADAYLARI
# ============================================================


def detect_horizon_y(
    lightness: np.ndarray,
    row_threshold: int = 35,
    padding: int = 5,
) -> int:
    """Üstteki siyah gökyüzünü çalışma alanından çıkarır."""
    height = lightness.shape[0]
    row_medians = np.median(lightness, axis=1)

    y = 0
    while y < height and row_medians[y] < row_threshold:
        y += 1

    if y >= 5:
        return min(height - 1, y + padding)

    return 0


def build_foreground_mask(
    image: np.ndarray,
    cluster_count: int = 5,
) -> tuple[np.ndarray, int]:
    """Baskın zemin renginden farklı bölgeleri ön plan maskesine çevirir."""
    height, width = image.shape[:2]
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lightness = lab[:, :, 0]

    roi_start_y = detect_horizon_y(lightness)
    roi = lab[roi_start_y:].astype(np.float32)
    pixels = roi.reshape(-1, 3)

    if len(pixels) == 0:
        return np.zeros((height, width), dtype=np.uint8), roi_start_y

    max_samples = 30_000
    if len(pixels) > max_samples:
        rng = np.random.default_rng(0)
        indices = rng.choice(len(pixels), max_samples, replace=False)
        samples = np.ascontiguousarray(pixels[indices], dtype=np.float32)
    else:
        samples = np.ascontiguousarray(pixels, dtype=np.float32)

    actual_cluster_count = min(cluster_count, len(samples))
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.5,
    )

    _, labels, centers = cv2.kmeans(
        samples,
        actual_cluster_count,
        None,
        criteria,
        5,
        cv2.KMEANS_PP_CENTERS,
    )

    cluster_sizes = np.bincount(
        labels.ravel(),
        minlength=actual_cluster_count,
    )
    background_center = centers[int(np.argmax(cluster_sizes))]

    color_distance = np.sqrt(
        np.sum((roi - background_center) ** 2, axis=2)
    )

    distance_8bit = cv2.normalize(
        color_distance,
        None,
        0,
        255,
        cv2.NORM_MINMAX,
    ).astype(np.uint8)

    _, roi_mask = cv2.threshold(
        distance_8bit,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )

    roi_mask = cv2.morphologyEx(
        roi_mask,
        cv2.MORPH_OPEN,
        np.ones((5, 5), dtype=np.uint8),
    )
    roi_mask = cv2.morphologyEx(
        roi_mask,
        cv2.MORPH_CLOSE,
        np.ones((11, 11), dtype=np.uint8),
    )

    full_mask = np.zeros((height, width), dtype=np.uint8)
    full_mask[roi_start_y:] = roi_mask
    return full_mask, roi_start_y


def split_regions_with_watershed(
    image: np.ndarray,
    foreground_mask: np.ndarray,
) -> list[dict]:
    """Birbirine gölgeyle bağlanan taş bölgelerini ayırır."""
    height, width = foreground_mask.shape

    opened = cv2.morphologyEx(
        foreground_mask,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
    )
    distance = cv2.distanceTransform(opened, cv2.DIST_L2, 5)

    markers = np.zeros((height, width), dtype=np.int32)
    next_label = 1

    component_count, components, stats, _ = cv2.connectedComponentsWithStats(
        opened
    )

    for component_id in range(1, component_count):
        _x, _y, _w, _h, area = stats[component_id]
        if area < 0.0004 * height * width:
            continue

        component_mask = components == component_id
        local_distance = distance * component_mask.astype(np.float32)
        maximum_distance = float(local_distance.max())

        if maximum_distance < 2:
            continue

        sure_foreground = (
            local_distance > 0.35 * maximum_distance
        ).astype(np.uint8) * 255

        sure_foreground = cv2.morphologyEx(
            sure_foreground,
            cv2.MORPH_OPEN,
            np.ones((5, 5), dtype=np.uint8),
        )

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

    regions: list[dict] = []
    for label in range(1, background_label):
        region_mask = (watershed_result == label).astype(np.uint8) * 255
        area = cv2.countNonZero(region_mask)
        if area == 0:
            continue

        contours, _ = cv2.findContours(
            region_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            continue

        contour = max(contours, key=cv2.contourArea)
        regions.append(
            {
                "mask": region_mask,
                "box": cv2.boundingRect(contour),
                "area": area,
            }
        )

    return regions


def is_valid_region(
    region: dict,
    image_shape: tuple[int, ...],
    roi_start_y: int,
) -> bool:
    """Aşırı küçük, ince veya kenara yapışık sahte bölgeleri eler."""
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
    minimum_center_y = roi_start_y + MIN_CENTER_Y_RATIO * (
        height - roi_start_y
    )
    return center_y >= minimum_center_y


# ============================================================
# ADAY TAŞIN RENGİNİ VE KOYULUĞUNU ÖLÇME
# ============================================================


def grabcut_candidate(
    image: np.ndarray,
    box: tuple[int, int, int, int],
) -> np.ndarray | None:
    """Aday kutudaki taşı GrabCut ile zeminden ayırır."""
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
        (grabcut_mask == cv2.GC_FGD)
        | (grabcut_mask == cv2.GC_PR_FGD),
        255,
        0,
    ).astype(np.uint8)

    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        foreground
    )
    if component_count <= 1:
        return None

    expected_center = np.array(
        [inner_x + inner_width / 2, inner_y + inner_height / 2],
        dtype=float,
    )

    best_component_id: int | None = None
    best_component_score = -1.0

    for component_id in range(1, component_count):
        area = stats[component_id, cv2.CC_STAT_AREA]
        if area < 100:
            continue

        center_distance = np.linalg.norm(
            centroids[component_id] - expected_center
        )
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


def evaluate_candidate(image: np.ndarray, region: dict) -> dict | None:
    """Küçük skor = daha koyu ve siyaha/griye daha yakın taş."""
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

    # Alt kısımdaki koyu gölgeyi ölçümden çıkar.
    shadow_cut_y = int(minimum_y + 0.80 * (maximum_y - minimum_y + 1))
    evaluation_mask[shadow_cut_y:maximum_y + 1, :] = 0

    eroded_mask = cv2.erode(
        evaluation_mask,
        np.ones((5, 5), dtype=np.uint8),
    )
    if cv2.countNonZero(eroded_mask) > 100:
        evaluation_mask = eroded_mask

    selected_lightness = lab[:, :, 0][evaluation_mask > 0]
    if len(selected_lightness) < 100:
        return None

    percentile_20, percentile_80 = np.percentile(
        selected_lightness,
        [20, 80],
    )
    trimmed_pixels = selected_lightness[
        (selected_lightness >= percentile_20)
        & (selected_lightness <= percentile_80)
    ]

    brightness = float(
        np.mean(trimmed_pixels)
        if len(trimmed_pixels)
        else np.mean(selected_lightness)
    )

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


def object_box_from_mask(
    mask: np.ndarray,
    fallback_box: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    """
    GrabCut maskesindeki yatay gölge uzantısını azaltarak taş gövdesinin
    kutusunu tahmin eder.
    """
    y_coordinates, x_coordinates = np.where(mask > 0)
    if len(x_coordinates) < 100:
        return fallback_box

    minimum_y = int(y_coordinates.min())
    maximum_y = int(y_coordinates.max())
    object_height = maximum_y - minimum_y + 1

    # Gölge çoğunlukla taşın alt bölümünde yatay olarak uzar.
    # Üst %62'lik bölüm, taş gövdesinin yatay sınırlarını daha iyi verir.
    row_indices = np.indices(mask.shape)[0]
    upper_part = (mask > 0) & (
        row_indices < minimum_y + 0.62 * object_height
    )

    upper_y, upper_x = np.where(upper_part)
    if len(upper_x) < 100:
        return fallback_box

    minimum_x = int(upper_x.min())
    maximum_x = int(upper_x.max())

    horizontal_margin = max(2, int(0.04 * (maximum_x - minimum_x + 1)))
    minimum_x = max(0, minimum_x - horizontal_margin)
    maximum_x = min(mask.shape[1] - 1, maximum_x + horizontal_margin)

    column_indices = np.indices(mask.shape)[1]
    restricted_mask = (mask > 0) & (column_indices >= minimum_x) & (
        column_indices <= maximum_x
    )

    restricted_y, restricted_x = np.where(restricted_mask)
    if len(restricted_x) < 100:
        return fallback_box

    minimum_y = int(restricted_y.min())
    maximum_y = int(restricted_y.max())

    return (
        minimum_x,
        minimum_y,
        maximum_x - minimum_x + 1,
        maximum_y - minimum_y + 1,
    )


# ============================================================
# KARE KUTU VE KOORDİNAT
# ============================================================


def make_square_box(
    box: tuple[int, int, int, int],
    image_shape: tuple[int, ...],
) -> tuple[int, int, int, int, int, int]:
    x, y, box_width, box_height = box
    image_height, image_width = image_shape[:2]

    center_x = x + box_width // 2
    center_y = y + box_height // 2
    square_size = max(
        2,
        int(math.ceil(max(box_width, box_height) * (1 + BOX_MARGIN_RATIO))),
    )

    half_size = square_size // 2
    x1 = max(0, center_x - half_size)
    y1 = max(0, center_y - half_size)
    x2 = min(image_width - 1, x1 + square_size - 1)
    y2 = min(image_height - 1, y1 + square_size - 1)

    x1 = max(0, x2 - square_size + 1)
    y1 = max(0, y2 - square_size + 1)

    return x1, y1, x2, y2, center_x, center_y


def detect_darkest_stone(
    image: np.ndarray,
) -> tuple[dict | None, np.ndarray]:
    foreground_mask, roi_start_y = build_foreground_mask(image)
    regions = split_regions_with_watershed(image, foreground_mask)

    regions = [
        region
        for region in regions
        if is_valid_region(region, image.shape, roi_start_y)
    ]

    if not regions:
        return None, foreground_mask

    largest_area = max(region["area"] for region in regions)
    regions = [
        region
        for region in regions
        if region["area"] >= 0.12 * largest_area
    ]

    candidates = []
    for region in regions:
        candidate = evaluate_candidate(image, region)
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        return None, foreground_mask

    darkest_candidate = min(candidates, key=lambda item: item["score"])
    object_box = object_box_from_mask(
        darkest_candidate["mask"],
        darkest_candidate["region"]["box"],
    )
    darkest_candidate["object_box"] = object_box
    darkest_candidate["square_box"] = make_square_box(
        object_box,
        image.shape,
    )
    return darkest_candidate, foreground_mask


# ============================================================
# KLASÖRDEKİ TÜM GÖRSELLERİ İŞLE
# ============================================================


def main() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if SAVE_DEBUG_MASKS:
        DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    if not INPUT_DIR.exists():
        INPUT_DIR.mkdir(parents=True, exist_ok=True)
        print(f"[BİLGİ] Girdi klasörü oluşturuldu: {INPUT_DIR}")
        print("Görselleri input_images klasörüne koyup kodu tekrar çalıştır.")
        return

    image_paths = sorted(
        path
        for path in INPUT_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS
    )

    if not image_paths:
        print(f"[HATA] İşlenecek görüntü bulunamadı: {INPUT_DIR}")
        return

    csv_path = OUTPUT_DIR / "coordinates.csv"

    with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "image",
                "center_x",
                "center_y",
                "x1",
                "y1",
                "x2",
                "y2",
                "brightness",
                "chroma",
                "selection_score",
            ]
        )

        for image_path in image_paths:
            print(f"\n[İŞLENİYOR] {image_path.name}")
            image = unicode_imread(image_path)

            if image is None:
                print(f"[HATA] Görüntü okunamadı: {image_path.name}")
                continue

            result, debug_mask = detect_darkest_stone(image)

            if SAVE_DEBUG_MASKS:
                debug_path = DEBUG_DIR / f"{image_path.stem}_mask.png"
                unicode_imwrite(debug_path, debug_mask)

            if result is None:
                print(f"[UYARI] Uygun taş bulunamadı: {image_path.name}")
                continue

            x1, y1, x2, y2, center_x, center_y = result["square_box"]
            annotated = image.copy()

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                4,
            )
            cv2.circle(
                annotated,
                (center_x, center_y),
                8,
                (0, 0, 255),
                -1,
            )

            label = f"Dark stone: ({center_x}, {center_y})"
            cv2.putText(
                annotated,
                label,
                (x1, max(35, y1 - 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            output_path = OUTPUT_DIR / f"{image_path.stem}_detected.jpg"
            if not unicode_imwrite(output_path, annotated):
                print(f"[HATA] Sonuç kaydedilemedi: {output_path.name}")
                continue

            writer.writerow(
                [
                    image_path.name,
                    center_x,
                    center_y,
                    x1,
                    y1,
                    x2,
                    y2,
                    round(result["brightness"], 3),
                    round(result["chroma"], 3),
                    round(result["score"], 3),
                ]
            )

            print(
                f"[OK] Merkez=({center_x}, {center_y}) | "
                f"Kutu=({x1}, {y1})-({x2}, {y2}) | "
                f"Skor={result['score']:.2f}"
            )

    print("\nİşlem tamamlandı.")
    print(f"Sonuç klasörü: {OUTPUT_DIR}")
    print(f"Koordinat dosyası: {csv_path}")


if __name__ == "__main__":
    main()