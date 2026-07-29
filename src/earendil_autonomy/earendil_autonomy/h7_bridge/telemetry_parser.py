"""Pure-Python parsers for the STM32H723 line-oriented telemetry protocol."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Mapping


_MOTOR_PATTERN = re.compile(
    r"(?:\[INFO\]\s*)?\[TEL\]\[(FL|FR|RL|RR)\]\s+(RPM:.*)$"
)


@dataclass(frozen=True)
class MotorRecord:
    motor: str
    fields: Mapping[str, str]


def parse_key_values(payload: str) -> dict[str, str]:
    """Parse comma-separated ``KEY:VALUE`` fields without losing strings."""
    result: dict[str, str] = {}
    for token in payload.split(","):
        if ":" not in token:
            continue
        key, value = token.split(":", 1)
        key = key.strip()
        if key:
            result[key] = value.strip()
    return result


def parse_record(line: str, marker: str) -> dict[str, str] | None:
    """Return fields after a record marker such as ``MPU_GYRO,``."""
    index = line.find(marker)
    if index < 0:
        return None
    return parse_key_values(line[index + len(marker) :])


def parse_motor_record(line: str) -> MotorRecord | None:
    """Parse a validated H7-tagged compact F411 telemetry record."""
    match = _MOTOR_PATTERN.search(line.strip())
    if match is None:
        return None

    fields = parse_key_values(match.group(2))
    if not all(key in fields for key in ("RPM", "PWM_ACT", "RXB")):
        return None

    numeric_fields = (
        "RPM",
        "T",
        "D",
        "APP_PH",
        "SP",
        "BRAKE",
        "FC",
        "H",
        "PWM_SET",
        "PWM_ACT",
        "QDROP",
        "RXB",
    )
    for key in numeric_fields:
        if key in fields and not re.fullmatch(r"-?\d+", fields[key]):
            return None

    if fields.get("DIR", "N") not in ("F", "R", "N"):
        return None

    return MotorRecord(motor=match.group(1), fields=fields)


def as_int(fields: Mapping[str, str], key: str, default: int = 0) -> int:
    try:
        value = fields[key].strip()
        signless = value[1:] if value[:1] in ("+", "-") else value
        base = 16 if signless.lower().startswith("0x") else 10
        return int(value, base)
    except (KeyError, TypeError, ValueError):
        return default


def has_valid_int_fields(fields: Mapping[str, str], required: tuple[str, ...]) -> bool:
    """Return true only when every required field is a decimal integer."""
    for key in required:
        value = fields.get(key)
        if value is None or re.fullmatch(r"[+-]?\d+", value.strip()) is None:
            return False
    return True
