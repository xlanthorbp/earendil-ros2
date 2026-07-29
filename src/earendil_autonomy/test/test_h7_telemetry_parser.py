from earendil_autonomy.h7_bridge.telemetry_parser import (
    as_int,
    has_valid_int_fields,
    parse_motor_record,
    parse_record,
)


def test_parses_split_imu_records():
    fields = parse_record(
        "[INFO] MPU_GYRO,GX:1000,GY:-2000,GZ:0,TC:2534,OK:1",
        "MPU_GYRO,",
    )
    assert fields is not None
    assert as_int(fields, "GX") == 1000
    assert as_int(fields, "GY") == -2000
    assert as_int(fields, "TC") == 2534


def test_parses_magnetometer_string_and_hex_fields():
    fields = parse_record(
        "[INFO] MAG_IMU,MX:12,MY:-34,MZ:56,MX_UTX100:40,MY_UTX100:-113,"
        "MZ_UTX100:186,STATUS:0x01,STATE:ONLINE,OK:1",
        "MAG_IMU,",
    )
    assert fields is not None
    assert fields["STATE"] == "ONLINE"
    assert as_int(fields, "STATUS") == 1
    assert as_int(fields, "MY_UTX100") == -113


def test_integer_parser_accepts_decimal_leading_zeroes_and_hex():
    fields = {"DEC": "08", "HEX": "0x0A", "NEG_HEX": "-0x0A"}
    assert as_int(fields, "DEC") == 8
    assert as_int(fields, "HEX") == 10
    assert as_int(fields, "NEG_HEX") == -10


def test_valid_integer_fields_reject_missing_or_malformed_imu_values():
    required = ("GX", "GY", "GZ", "TC", "OK")
    assert has_valid_int_fields(
        {"GX": "1", "GY": "-2", "GZ": "+3", "TC": "2500", "OK": "1"},
        required,
    )
    assert not has_valid_int_fields(
        {"GX": "1", "GY": "bad", "GZ": "3", "TC": "2500", "OK": "1"},
        required,
    )
    assert not has_valid_int_fields(
        {"GX": "1", "GY": "2", "GZ": "3", "OK": "1"}, required
    )


def test_parses_all_four_motor_tags():
    for motor in ("FL", "FR", "RL", "RR"):
        record = parse_motor_record(
            f"[INFO] [TEL][{motor}] "
            "RPM:60,T:60,D:0,DIR:F,APP_PH:2,SP:1,BRAKE:0,FC:0,H:4,"
            "PWM_SET:100,PWM_ACT:95,QDROP:0,RXB:23695"
        )
        assert record is not None
        assert record.motor == motor
        assert record.fields["DIR"] == "F"
        assert as_int(record.fields, "RPM") == 60


def test_rejects_malformed_or_incomplete_motor_telemetry():
    assert parse_motor_record("[INFO] [TEL][FL] RPM:bad,PWM_ACT:0,RXB:1") is None
    assert parse_motor_record("[INFO] [TEL][FL] RPM:1,DIR:F,RXB:1") is None
    assert (
        parse_motor_record("[INFO] [TEL][FL] RPM:1,DIR:X,PWM_ACT:1,RXB:1")
        is None
    )
