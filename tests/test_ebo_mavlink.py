import math
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "app"))

from ebo_mavlink import (FrameBuilder, MAVLinkV1Stream, TimedTask,
                         decode_telemetry, parse_parameter_value,
                         parse_file_server_info, parse_timed_task)


def test_stream_handles_fragmentation_noise_and_multiple_frames():
    builder = FrameBuilder()
    first = builder.frame(205, b"\x32")
    second = builder.frame(213, struct.pack("<hh", -3, 9))
    stream = MAVLinkV1Stream()
    assert stream.feed(b"noise" + first[:4]) == []
    frames = stream.feed(first[4:] + second)
    assert [frame.message_id for frame in frames] == [205, 213]
    assert [frame.sequence for frame in frames] == [0, 1]


def test_stream_rejects_bad_known_crc_and_resynchronizes():
    builder = FrameBuilder()
    bad = bytearray(builder.frame(205, b"\x01"))
    bad[-1] ^= 0xFF
    good = builder.frame(205, b"\x02")
    frames = MAVLinkV1Stream().feed(bytes(bad) + good)
    assert len(frames) == 1
    assert frames[0].payload == b"\x02"


def test_parameter_request_set_and_value_round_trip():
    builder = FrameBuilder()
    request = MAVLinkV1Stream().feed(builder.param_request("audio", "volume"))[0]
    assert request.message_id == 226
    assert struct.unpack_from("<hBB", request.payload) == (-1, 255, 3)
    assert request.payload[4:].rstrip(b"\0") == b"audio-volume"

    set_frame = MAVLinkV1Stream().feed(builder.param_set("control", "speed", 72, 6))[0]
    assert set_frame.message_id == 229
    assert math.isclose(struct.unpack_from("<f", set_frame.payload)[0], 72)
    assert set_frame.payload[-1] == 6

    payload = struct.pack("<fHH", 72.0, 14, 3) + b"control-speed".ljust(32, b"\0") + b"\x06"
    value = parse_parameter_value(payload)
    assert (value.name, value.value, value.parameter_type, value.count, value.index) == (
        "control-speed", 72.0, 6, 14, 3)


def test_timed_task_set_and_item_round_trip():
    task = TimedTask(task_id=42, task_type=2, enabled=True, weekdays=0b0011111,
                     start_hour=13, start_minute=4, end_hour=13, end_minute=7,
                     security_type=3, security_disabled=1, security_air_mode=2)
    frame = MAVLinkV1Stream().feed(FrameBuilder().timed_task_set(task))[0]
    assert frame.message_id == 155
    assert len(frame.payload) == 35

    item_payload = frame.payload[:28] + bytes((2, 1, 0)) + frame.payload[29:]
    parsed = parse_timed_task(item_payload)
    assert parsed.task_id == 42
    assert parsed.task_type == 2
    assert parsed.count == 1 and parsed.index == 0
    assert parsed.start_minute == 4 and parsed.end_minute == 7


def test_do_not_disturb_task_and_file_server_frames():
    builder = FrameBuilder()
    dnd = TimedTask(task_type=3, weekdays=0x7f, start_hour=0, start_minute=0,
                    end_hour=23, end_minute=59)
    assert MAVLinkV1Stream().feed(builder.timed_task_set(dnd))[0].payload[28] == 3
    assert MAVLinkV1Stream().feed(builder.file_server_open())[0].message_id == 40
    assert MAVLinkV1Stream().feed(builder.file_server_close())[0].message_id == 41

    payload = struct.pack("<IB", 4567, 0) + b"192.168.10.9\0\0\0\0" + bytes(range(32))
    assert parse_file_server_info(payload) == {
        "port": 4567, "status": 0, "host": "192.168.10.9", "password": bytes(range(32))}


def test_battery_and_motion_telemetry_decode():
    builder = FrameBuilder()
    battery_payload = struct.pack("<4fBB", 8.1, -0.2, 8.0, 8.4, 83, 1)
    battery = MAVLinkV1Stream().feed(builder.frame(207, battery_payload))[0]
    decoded = decode_telemetry(battery)["battery"]
    assert decoded["percent"] == 83
    assert decoded["charge_status"] == 1
    assert math.isclose(decoded["measured_voltage"], 8.1, rel_tol=1e-6)

    motor = MAVLinkV1Stream().feed(builder.frame(210, struct.pack("<hhB", -12, 13, 2)))[0]
    assert decode_telemetry(motor) == {
        "motor_speed": {"right": -12, "left": 13, "control_mode": 2}}
