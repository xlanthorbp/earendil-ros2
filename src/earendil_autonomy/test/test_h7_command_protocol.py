from earendil_autonomy.h7_bridge.command_protocol import (
    is_h7_restart,
    is_pc_link_timeout,
    mode_command,
    mode_from_line,
    normalize_mode,
)


def test_normalize_mode_aliases():
    assert normalize_mode("DISARM") == "disarm"
    assert normalize_mode("mode manual") == "manual"
    assert normalize_mode(" auto ") == "autonomous"
    assert normalize_mode("mode autonomous") == "autonomous"
    assert normalize_mode("mode speed") is None


def test_mode_command_uses_firmware_protocol():
    assert mode_command("disarm") == "mode disarm"
    assert mode_command("manual") == "mode manual"
    assert mode_command("auto") == "mode autonomous"


def test_mode_from_acknowledgement_and_query():
    assert mode_from_line("[INFO] [MODE] AUTONOMOUS active") == "autonomous"
    assert mode_from_line("[MODE] DISARM active, motion commands locked") == "disarm"
    assert mode_from_line("[INFO] Rover mode: MANUAL") == "manual"
    assert mode_from_line("[BOOT] Operating mode: DISARM (motion locked)") == "disarm"
    assert mode_from_line("unrelated telemetry") is None


def test_pc_link_timeout_detection_is_specific():
    assert is_pc_link_timeout(
        "[PC_LINK] TIMEOUT,AGE_MS:2001,ACTION:STOP_DISARM"
    )
    assert not is_pc_link_timeout("[PC_LINK] RECOVERED")
    assert not is_pc_link_timeout("[MODE] DISARM active")


def test_h7_restart_detection():
    assert is_h7_restart("[BOOT] H723 rover main controller started")
    assert is_h7_restart("[BOOT] Operating mode: DISARM (motion locked)")
    assert not is_h7_restart("[MODE] DISARM active")
