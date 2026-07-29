import math

from earendil_autonomy.h7_bridge.magnetometer_math import (
    CircularFilter,
    build_minmax_calibration,
    calculate_heading,
    calibration_from_mapping,
    valid_mag_sample,
)


def angular_error(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def test_provided_calibration_format_is_compatible():
    calibration = calibration_from_mapping(
        {
            "source": "QMC5883L raw MX/MY/MZ",
            "offset": {"x": 20.0, "y": -316.0, "z": -100.0},
            "scale": {"x": 0.97, "y": 1.04, "z": 0.98},
            "radius_ratio": 1.07,
        }
    )
    calibrated = calibration.apply(120, -216, 0)
    assert calibrated == (97.0, 104.0, 98.0)


def test_default_heading_matches_original_inverted_y_algorithm():
    assert calculate_heading((100.0, 0.0, 0.0)) == 0.0
    assert calculate_heading((0.0, 100.0, 0.0)) == 270.0
    assert calculate_heading((-100.0, 0.0, 0.0)) == 180.0
    assert calculate_heading((0.0, -100.0, 0.0)) == 90.0


def test_circular_filter_does_not_jump_at_north_wrap():
    heading_filter = CircularFilter(alpha=0.5)
    first = heading_filter.update(359.0)
    second = heading_filter.update(1.0)
    assert angular_error(first, 359.0) < 0.01
    assert angular_error(second, 0.0) < 0.1


def test_minmax_builder_recovers_offsets_scales_and_ratio():
    result = build_minmax_calibration(
        (-1246, -1497, -1352),
        (1286, 865, 1152),
    )
    assert result["offset"] == {"x": 20.0, "y": -316.0, "z": -100.0}
    assert math.isclose(result["radius_ratio"], 1266.0 / 1181.0)
    assert math.isclose(result["scale"]["x"], (1266 + 1181 + 1252) / 3 / 1266)


def test_rejects_invalid_mag_samples():
    assert valid_mag_sample({"MX": "1", "MY": "2", "MZ": "3", "OK": "1"}) is None
    assert (
        valid_mag_sample(
            {
                "MX": "100",
                "MY": "200",
                "MZ": "300",
                "DRDY": "1",
                "OVFL": "1",
                "OK": "1",
            }
        )
        is None
    )
    assert valid_mag_sample(
        {
            "MX": "100",
            "MY": "-200",
            "MZ": "300",
            "DRDY": "1",
            "OVFL": "0",
            "OK": "1",
        }
    ) == (100, -200, 300)
