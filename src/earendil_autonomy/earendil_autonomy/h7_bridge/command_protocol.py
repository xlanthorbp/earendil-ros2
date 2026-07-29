"""Helpers for the STM32H723 operating-mode terminal protocol."""

from __future__ import annotations

import re


VALID_MODES = ("disarm", "manual", "autonomous")

_MODE_ALIASES = {
    "disarm": "disarm",
    "manual": "manual",
    "auto": "autonomous",
    "autonomous": "autonomous",
    "mode disarm": "disarm",
    "mode manual": "manual",
    "mode auto": "autonomous",
    "mode autonomous": "autonomous",
}

_ACTIVE_MODE_PATTERN = re.compile(
    r"\[MODE\]\s+(DISARM|MANUAL|AUTONOMOUS)\s+active", re.IGNORECASE
)
_QUERY_MODE_PATTERN = re.compile(
    r"Rover\s+mode:\s*(DISARM|MANUAL|AUTONOMOUS)", re.IGNORECASE
)
_BOOT_MODE_PATTERN = re.compile(
    r"Operating\s+mode:\s*(DISARM|MANUAL|AUTONOMOUS)", re.IGNORECASE
)


def normalize_mode(value: str) -> str | None:
    """Normalize a ROS mode request to one of ``VALID_MODES``."""
    clean = " ".join(value.strip().lower().split())
    return _MODE_ALIASES.get(clean)


def mode_command(mode: str) -> str:
    """Return the exact terminal command accepted by the H7 firmware."""
    normalized = normalize_mode(mode)
    if normalized is None:
        raise ValueError(f"unsupported rover mode: {mode!r}")
    return f"mode {normalized}"


def mode_from_line(line: str) -> str | None:
    """Extract the operating mode from H7 acknowledgement/query output."""
    for pattern in (_ACTIVE_MODE_PATTERN, _QUERY_MODE_PATTERN, _BOOT_MODE_PATTERN):
        match = pattern.search(line)
        if match is not None:
            return match.group(1).lower()
    return None


def is_pc_link_timeout(line: str) -> bool:
    """Return true for the firmware's watchdog STOP+DISARM report."""
    upper = line.upper()
    return "[PC_LINK]" in upper and "TIMEOUT" in upper and "STOP_DISARM" in upper


def is_h7_restart(line: str) -> bool:
    """Return true for an H7 boot banner that implies startup in DISARM."""
    upper = line.upper()
    return (
        "H723 ROVER MAIN CONTROLLER STARTED" in upper
        or "OPERATING MODE: DISARM (MOTION LOCKED)" in upper
    )
