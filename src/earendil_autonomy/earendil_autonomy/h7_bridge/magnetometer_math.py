"""Calibration and heading math shared by the ROS magnetometer nodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

from .telemetry_parser import as_int


AXES = ("x", "y", "z")
RAW_QMC_LSB_TO_TESLA = 1.0e-6 / 30.0


@dataclass(frozen=True)
class MagnetometerCalibration:
    offset_x: float
    offset_y: float
    offset_z: float
    scale_x: float
    scale_y: float
    scale_z: float
    source: str = "QMC5883L raw MX/MY/MZ"
    radius_ratio: float | None = None

    def apply(self, mx: int, my: int, mz: int) -> tuple[float, float, float]:
        return (
            (mx - self.offset_x) * self.scale_x,
            (my - self.offset_y) * self.scale_y,
            (mz - self.offset_z) * self.scale_z,
        )


class CircularFilter:
    """First-order low-pass filter that remains correct around 0/360 deg."""

    def __init__(self, alpha: float) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.reset()

    def reset(self) -> None:
        self._initialized = False
        self._sin = 0.0
        self._cos = 1.0

    def update(self, heading_deg: float) -> float:
        radians = math.radians(heading_deg)
        current_sin = math.sin(radians)
        current_cos = math.cos(radians)
        if not self._initialized:
            self._sin = current_sin
            self._cos = current_cos
            self._initialized = True
        else:
            self._sin = (1.0 - self.alpha) * self._sin + self.alpha * current_sin
            self._cos = (1.0 - self.alpha) * self._cos + self.alpha * current_cos
        return math.degrees(math.atan2(self._sin, self._cos)) % 360.0


def _finite_number(section: Mapping, key: str) -> float:
    try:
        value = float(section[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"missing or invalid calibration value: {key}") from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite calibration value: {key}")
    return value


def calibration_from_mapping(data: Mapping) -> MagnetometerCalibration:
    try:
        offsets = data["offset"]
        scales = data["scale"]
    except (KeyError, TypeError) as exc:
        raise ValueError("calibration JSON must contain offset and scale") from exc
    if not isinstance(offsets, Mapping) or not isinstance(scales, Mapping):
        raise ValueError("offset and scale must be JSON objects")

    values = {
        "offset_x": _finite_number(offsets, "x"),
        "offset_y": _finite_number(offsets, "y"),
        "offset_z": _finite_number(offsets, "z"),
        "scale_x": _finite_number(scales, "x"),
        "scale_y": _finite_number(scales, "y"),
        "scale_z": _finite_number(scales, "z"),
    }
    if any(values[key] <= 0.0 for key in ("scale_x", "scale_y", "scale_z")):
        raise ValueError("all calibration scale values must be positive")

    radius_ratio = data.get("radius_ratio")
    if radius_ratio is not None:
        radius_ratio = float(radius_ratio)
        if not math.isfinite(radius_ratio) or radius_ratio < 1.0:
            raise ValueError("radius_ratio must be finite and >= 1")

    return MagnetometerCalibration(
        **values,
        source=str(data.get("source", "QMC5883L raw MX/MY/MZ")),
        radius_ratio=radius_ratio,
    )


def load_calibration(path: str | Path) -> MagnetometerCalibration:
    calibration_path = Path(path).expanduser()
    try:
        data = json.loads(calibration_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"calibration file not found: {calibration_path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read calibration file: {exc}") from exc
    try:
        return calibration_from_mapping(data)
    except ValueError as exc:
        raise RuntimeError(f"invalid calibration file: {exc}") from exc


def valid_mag_sample(fields: Mapping[str, str]) -> tuple[int, int, int] | None:
    if as_int(fields, "OK") != 1:
        return None
    if as_int(fields, "DRDY") != 1:
        return None
    if as_int(fields, "OVFL") != 0:
        return None
    if any(axis not in fields for axis in ("MX", "MY", "MZ")):
        return None

    try:
        sample = tuple(int(fields[key], 10) for key in ("MX", "MY", "MZ"))
    except (TypeError, ValueError):
        return None
    if sum(abs(value) for value in sample) < 30:
        return None
    if any(value in (-32768, 32767) for value in sample):
        return None
    if sample == (-1, -1, -1):
        return None
    return sample


def calculate_heading(
    calibrated_xyz: Sequence[float],
    *,
    offset_deg: float = 0.0,
    invert_x: bool = False,
    invert_y: bool = True,
    swap_xy: bool = False,
    minimum_horizontal: float = 5.0,
) -> float | None:
    calibrated_x, calibrated_y, _ = calibrated_xyz
    heading_x, heading_y = (
        (calibrated_y, calibrated_x) if swap_xy else (calibrated_x, calibrated_y)
    )
    if invert_x:
        heading_x = -heading_x
    if invert_y:
        heading_y = -heading_y
    if heading_x * heading_x + heading_y * heading_y < minimum_horizontal**2:
        return None
    heading = (math.degrees(math.atan2(heading_y, heading_x)) + offset_deg) % 360.0
    return heading if math.isfinite(heading) else None


def build_minmax_calibration(
    minimum: Sequence[float],
    maximum: Sequence[float],
    *,
    minimum_axis_radius: float = 50.0,
) -> dict:
    if len(minimum) != 3 or len(maximum) != 3:
        raise ValueError("minimum and maximum must contain three axes")
    radii = [(float(high) - float(low)) / 2.0 for low, high in zip(minimum, maximum)]
    if any(not math.isfinite(radius) for radius in radii):
        raise ValueError("axis extrema must be finite")
    if min(radii) < minimum_axis_radius:
        raise ValueError("one or more axes were not covered sufficiently")

    offsets = [
        (float(high) + float(low)) / 2.0 for low, high in zip(minimum, maximum)
    ]
    average_radius = sum(radii) / 3.0
    scales = [average_radius / radius for radius in radii]
    ratio = max(radii) / min(radii)
    return {
        "offset": dict(zip(AXES, offsets)),
        "scale": dict(zip(AXES, scales)),
        "radius": dict(zip(AXES, radii)),
        "radius_ratio": ratio,
    }
