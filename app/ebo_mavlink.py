"""Small MAVLink-v1 subset used by the EBO SE control channel.

The robot transports these frames inside its RDT data stream.  This module is
deliberately independent from pymavlink: Enabot's private messages are not in
the public dialect and the bridge only needs a compact, auditable subset.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import struct
import threading
from typing import Any


CRC_EXTRA = {
    40: 70, 41: 41, 42: 136, 43: 213, 44: 58, 45: 58,
    48: 246, 152: 76, 153: 250, 154: 88, 155: 84, 156: 208, 157: 195,
    163: 48, 166: 133, 167: 16, 200: 196, 202: 211, 204: 19,
    205: 42, 207: 222, 210: 100, 213: 117, 218: 84, 240: 9,
    226: 100, 227: 159, 228: 164, 229: 208,
}


def crc_x25(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in data + bytes((extra,)):
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class FrameBuilder:
    """Build MAVLink-v1 frames with a monotonically increasing sequence."""

    def __init__(self, system_id: int = 0, component_id: int = 0):
        self.system_id = system_id
        self.component_id = component_id
        self._sequence = 0
        self._lock = threading.Lock()

    def frame(self, message_id: int, payload: bytes, crc_extra: int | None = None) -> bytes:
        if not 0 <= message_id <= 255 or len(payload) > 255:
            raise ValueError("MAVLink-v1 message or payload out of range")
        extra = CRC_EXTRA.get(message_id) if crc_extra is None else crc_extra
        if extra is None:
            raise ValueError(f"unknown CRC extra for message {message_id}")
        with self._lock:
            sequence = self._sequence
            self._sequence = (sequence + 1) & 0xFF
        body = bytes((len(payload), sequence, self.system_id, self.component_id, message_id)) + payload
        return b"\xfe" + body + struct.pack("<H", crc_x25(body, extra))

    def param_request(self, group: str, key: str) -> bytes:
        name = _parameter_name(group, key)
        payload = struct.pack("<hBB", -1, 255, 3) + name
        return self.frame(226, payload)

    def param_list_request(self) -> bytes:
        return self.frame(227, bytes((255, 3)))

    def param_set(self, group: str, key: str, value: float, parameter_type: int = 11) -> bytes:
        payload = struct.pack("<fBB", float(value), 255, 3) + _parameter_name(group, key) + bytes((parameter_type,))
        return self.frame(229, payload)

    def timed_task_list(self, task_type: int) -> bytes:
        _task_type(task_type)
        return self.frame(153, bytes((task_type,)))

    def timed_task_read(self, task_type: int, index: int) -> bytes:
        _task_type(task_type)
        if not 0 <= index <= 255:
            raise ValueError("task index must be 0..255")
        return self.frame(152, bytes((task_type, index)))

    def timed_task_set(self, task: "TimedTask") -> bytes:
        task.validate()
        payload = struct.pack(
            "<7i7B", task.task_id, task.security_type, task.security_disabled,
            task.security_air_mode, 0, 0, 0, task.task_type, int(task.enabled),
            task.weekdays, task.start_hour, task.start_minute, task.end_hour,
            task.end_minute,
        )
        return self.frame(155, payload)

    def timed_task_command(self, task_id: int, task_type: int, request: int) -> bytes:
        _task_type(task_type)
        if not 0 <= request <= 255:
            raise ValueError("task request must be 0..255")
        return self.frame(156, struct.pack("<iBB", task_id, task_type, request))

    def file_server_open(self) -> bytes:
        return self.frame(40, bytes(36))

    def file_server_close(self) -> bytes:
        return self.frame(41, b"\0")


def _parameter_name(group: str, key: str) -> bytes:
    name = f"{group}-{key}".encode("ascii")
    if not name or len(name) > 32:
        raise ValueError("parameter name must be 1..32 ASCII bytes")
    return name.ljust(32, b"\0")


def _task_type(value: int) -> None:
    if value not in (1, 2, 3):
        raise ValueError("task type must be 1 (fixed point), 2 (cruise), or 3 (do not disturb)")


@dataclass(frozen=True)
class MAVFrame:
    message_id: int
    sequence: int
    system_id: int
    component_id: int
    payload: bytes


class MAVLinkV1Stream:
    """Fragment-safe decoder. Bad known frames are skipped and parsing resyncs."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[MAVFrame]:
        self._buffer.extend(data)
        result: list[MAVFrame] = []
        while True:
            start = self._buffer.find(0xFE)
            if start < 0:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 8:
                break
            length = self._buffer[1]
            total = length + 8
            if len(self._buffer) < total:
                break
            raw = bytes(self._buffer[:total])
            message_id = raw[5]
            extra = CRC_EXTRA.get(message_id)
            expected = struct.unpack_from("<H", raw, total - 2)[0]
            if extra is not None and crc_x25(raw[1:-2], extra) != expected:
                del self._buffer[0]
                continue
            result.append(MAVFrame(message_id, raw[2], raw[3], raw[4], raw[6:-2]))
            del self._buffer[:total]
        return result


@dataclass(frozen=True)
class ParameterValue:
    name: str
    value: float
    parameter_type: int
    count: int
    index: int


def parse_parameter_value(payload: bytes) -> ParameterValue:
    if len(payload) != 41:
        raise ValueError("invalid PARAM_VALUE payload")
    value, count, index = struct.unpack_from("<fHH", payload)
    name = payload[8:40].split(b"\0", 1)[0].decode("ascii")
    return ParameterValue(name, value, payload[40], count, index)


@dataclass(frozen=True)
class TimedTask:
    task_id: int = 0
    task_type: int = 2
    enabled: bool = True
    weekdays: int = 0x7F
    start_hour: int = 0
    start_minute: int = 0
    end_hour: int = 0
    end_minute: int = 3
    security_type: int = 0
    security_disabled: int = 0
    security_air_mode: int = 0
    count: int | None = None
    index: int | None = None

    def validate(self) -> None:
        _task_type(self.task_type)
        if not 0 <= self.weekdays <= 0x7F:
            raise ValueError("weekdays must be a 7-bit mask")
        for value, label, maximum in ((self.start_hour, "start_hour", 23),
                                      (self.end_hour, "end_hour", 23),
                                      (self.start_minute, "start_minute", 59),
                                      (self.end_minute, "end_minute", 59)):
            if not 0 <= value <= maximum:
                raise ValueError(f"{label} out of range")

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def parse_timed_task(payload: bytes) -> TimedTask:
    if len(payload) != 37:
        raise ValueError("invalid TIMED_TASK_ITEM payload")
    ints = struct.unpack_from("<7i", payload)
    task_type, count, index, enabled, weekdays, sh, sm, eh, em = struct.unpack_from("<9B", payload, 28)
    task = TimedTask(ints[0], task_type, bool(enabled), weekdays, sh, sm, eh, em,
                     ints[1], ints[2], ints[3], count, index)
    task.validate()
    return task


def parse_task_ack(payload: bytes) -> dict[str, int]:
    if len(payload) != 6:
        raise ValueError("invalid TIMED_TASK_ACK payload")
    task_id, task_type, result = struct.unpack("<iBB", payload)
    return {"task_id": task_id, "task_type": task_type, "result": result}


def parse_file_server_info(payload: bytes) -> dict[str, Any]:
    if len(payload) != 53:
        raise ValueError("invalid FILE_SERVER_INFO payload")
    port, status = struct.unpack_from("<IB", payload)
    host = payload[5:21].split(b"\0", 1)[0].decode("ascii")
    password = bytes(payload[21:53])
    return {"port": port, "status": status, "host": host, "password": password}


def decode_telemetry(frame: MAVFrame) -> dict[str, Any] | None:
    p, mid = frame.payload, frame.message_id
    try:
        if mid == 207 and len(p) == 18:
            measured, current, estimated, charge_voltage = struct.unpack_from("<4f", p)
            return {"battery": {"measured_voltage": measured, "current": current,
                    "estimated_voltage": estimated, "charge_voltage": charge_voltage,
                    "percent": p[16], "charge_status": p[17]}}
        if mid == 163 and len(p) == 16:
            boot, roll, pitch, yaw = struct.unpack("<I3f", p)
            return {"attitude": {"time_boot_ms": boot, "roll": roll, "pitch": pitch, "yaw": yaw}}
        if mid in (166, 167) and len(p) == 12:
            xyz = dict(zip(("x", "y", "z"), struct.unpack("<3f", p)))
            return {"acceleration" if mid == 166 else "gyroscope": xyz}
        if mid == 204 and len(p) == 5:
            return {"rangefinder": {"range": struct.unpack_from("<f", p)[0], "confidence": p[4]}}
        if mid == 205 and len(p) == 1:
            return {"speed_level": p[0]}
        if mid == 210 and len(p) == 5:
            right, left, mode = struct.unpack("<hhB", p)
            return {"motor_speed": {"right": right, "left": left, "control_mode": mode}}
        if mid == 213 and len(p) == 4:
            left, right = struct.unpack("<hh", p)
            return {"wheel_circles": {"left": left, "right": right}}
        if mid == 240 and len(p) == 10:
            right, left, right_otp, left_otp = struct.unpack("<ffBB", p)
            return {"motor_temperature": {"right": right, "left": left,
                    "right_overtemperature": bool(right_otp), "left_overtemperature": bool(left_otp)}}
        if mid == 45 and len(p) == 2:
            return {"camera": {"recording": bool(p[0]), "trigger_photo": bool(p[1])}}
        if mid == 48 and len(p) == 3:
            return {"mcu": {"system_mode": p[0], "auto_mode_type": p[1], "tracking": bool(p[2])}}
    except struct.error:
        return None
    return None
