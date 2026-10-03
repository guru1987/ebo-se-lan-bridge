import json
import socket
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ebo_local_responder import (Config, KEEPALIVE, RegistrationProtocol,
                                 reverse_transcode, transcode)


UID = b"ABCDEFGHIJKLMNOPQRST"
SOURCE = ("192.0.2.10", 45678)


def config():
    return Config(
        bind_ip="192.0.2.20",
        robot_ip=SOURCE[0],
        uid=UID,
        advertise_ip="192.0.2.20",
        allowed_sources=frozenset({SOURCE[0]}),
    )


def header(kind, size, command):
    return kind + struct.pack("<I", size) + command + b"\0" * 4


def request(kind):
    if kind == "master":
        plain = header(b"\x04\x02\x1d\x02", 38, b"\x07\x10\x18\0")
        plain += UID + b"redacted-master!" + b"\x06\x01"
        return 10240, transcode(plain)
    if kind == "probe":
        plain = header(b"\x04\x02\x1d\x02", 8, b"\x03\x80\x3f\0")
        plain += b"\x2a\0\0\0\x34\x12\x01\0"
        return 3478, transcode(plain)
    if kind == "login":
        plain = header(b"\x04\x02\x1d\x02", 112, b"\x20\x01\x14\0")
        plain += UID
        plain += b"\0" * (128 - len(plain))
        return 3478, transcode(plain)
    if kind == "register":
        plain = header(b"\x04\x02\x1d\x02", 32, b"\x22\x01\x14\0")
        plain += UID + b"SYNTHETC" + b"\x78\x56\x34\x12"
        return 3478, transcode(plain)
    if kind == "keepalive":
        return 3478, KEEPALIVE
    raise AssertionError(kind)


def test_transcode_round_trip_all_capture_packet_sizes():
    for size in (4, 24, 40, 44, 48, 54, 56, 126, 128):
        plain = bytes((index * 17 + 3) & 0xFF for index in range(size))
        assert reverse_transcode(transcode(plain)) == plain


def test_response_shapes_are_dynamic_and_request_bound():
    protocol = RegistrationProtocol(config())
    expected = {"master": 126, "probe": 40, "login": 44, "register": 56}
    for kind, response_length in expected.items():
        port, packet = request(kind)
        response = protocol.handle(port, packet, SOURCE)
        assert response is not None and len(response) == response_length
        plain = reverse_transcode(response)
        assert struct.unpack_from("<I", plain, 4)[0] == response_length - 16
        if kind != "probe":
            assert UID in plain

    register_plain = reverse_transcode(
        protocol.handle(3478, request("register")[1], SOURCE))
    assert register_plain[-4:] == b"\x78\x56\x34\x12"
    assert len(register_plain) == 56  # regression: never replay the old 78-byte packet
    assert socket.inet_aton(SOURCE[0]) in register_plain


def test_redacted_125_second_trace_never_emits_bad_registration_reply():
    fixture = json.loads((ROOT / "tests" / "fixtures" /
                          "local_responder_125s.json").read_text())
    protocol = RegistrationProtocol(config())
    for event in fixture["events"]:
        port, packet = request(event["kind"])
        response = protocol.handle(port, packet, SOURCE)
        expected = event["expect"]
        assert (None if response is None else len(response)) == expected
        assert response is None or len(response) != 78
    assert protocol.counters["dropped"] == 0
    assert protocol.counters["register"] == 5


def test_wrong_source_uid_and_unknown_packet_fail_closed():
    protocol = RegistrationProtocol(config())
    port, packet = request("register")
    assert protocol.handle(port, packet, ("192.0.2.99", SOURCE[1])) is None

    decoded = bytearray(reverse_transcode(packet))
    decoded[16:36] = b"XXXXXXXXXXXXXXXXXXXX"
    assert protocol.handle(port, transcode(bytes(decoded)), SOURCE) is None
    assert protocol.handle(port, b"not-kalay", SOURCE) is None
    assert protocol.counters["dropped"] == 3


def test_environment_config_rejects_placeholder_and_accepts_hairpin_sources():
    base = {
        "EBO_RESPONDER_BIND_IP": "192.0.2.20",
        "EBO_RESPONDER_ROBOT_IP": "192.0.2.10",
        "EBO_RESPONDER_ADVERTISE_IP": "192.0.2.20",
        "EBO_RESPONDER_ALLOWED_SOURCES": "192.0.2.1,192.0.2.10",
        "EBO_RESPONDER_UID": "REPLACE_WITH_20_CHAR_UID",
    }
    try:
        Config.from_environ(base)
    except ValueError as exc:
        assert "20-character UID" in str(exc)
    else:
        raise AssertionError("placeholder UID accepted")

    base["EBO_RESPONDER_UID"] = UID.decode()
    parsed = Config.from_environ(base)
    assert parsed.allowed_sources == frozenset({"192.0.2.1", "192.0.2.10"})
