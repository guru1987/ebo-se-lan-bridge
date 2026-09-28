"""Small client for the EBO SE's MAVLink file server.

ROLA asks for a temporary TCP endpoint over the normal RDT channel, logs in
with the returned 32-byte password, then speaks MAVLink FILE_TRANSFER_PROTOCOL
on that socket.  Keeping this separate from the AV session lets media browsing
and downloads coexist with video, hearing, and talkback.
"""
from __future__ import annotations

import socket
import struct
from pathlib import Path

from ebo_mavlink import FrameBuilder, MAVLinkV1Stream


MEDIA_DIRS = (
    "/DICM", "/RECORD", "/RECORD/AVI_EDIT", "/RECORD/PATROL",
    "/RECORD/FIXEDPOINT", "/RECORD/MOTIONDET", "/RECORD/TIMING",
    "/RECORD/AIDETECT", "/DICM/ELECPET", "/RECORD/ELECPET",
)


class MediaProtocolError(RuntimeError):
    pass


class EboMediaClient:
    def __init__(self, host: str, port: int, password: bytes, timeout: float = 8.0):
        if not host or not 0 < port < 65536 or len(password) != 32:
            raise ValueError("invalid EBO file-server details")
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.frames = MAVLinkV1Stream()
        self.builder = FrameBuilder()
        self.sequence = 0
        self._pending = []
        self.sock.sendall(self.builder.frame(43, password))
        login = self._next_frame(44)
        if not login.payload or login.payload[0] != 0:
            self.close()
            raise MediaProtocolError("EBO file-server login failed")

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def _next_frame(self, message_id: int):
        while True:
            for index, frame in enumerate(self._pending):
                if frame.message_id == message_id:
                    return self._pending.pop(index)
            data = self.sock.recv(65536)
            if not data:
                raise MediaProtocolError("EBO file server disconnected")
            self._pending.extend(self.frames.feed(data))

    def _request(self, opcode: int, data: bytes = b"", *, session: int = 0,
                 offset: int = 0, size: int | None = None):
        if len(data) > 239:
            raise ValueError("MAVLink FTP data exceeds 239 bytes")
        self.sequence = (self.sequence + 1) & 0xffff
        requested_size = len(data) if size is None else size
        inner = struct.pack("<HBBBBBBI", self.sequence, session, opcode,
                            requested_size, 0, 0, 0, offset) + data
        payload = b"\0\0\0" + inner.ljust(251, b"\0")
        self.sock.sendall(self.builder.frame(218, payload))

    def _response(self, request_opcode: int):
        while True:
            frame = self._next_frame(218)
            if len(frame.payload) != 254:
                continue
            inner = frame.payload[3:]
            seq, session, opcode, size, requested, burst, _pad, offset = struct.unpack_from(
                "<HBBBBBBI", inner)
            if requested != request_opcode:
                continue
            data = inner[12:12 + size]
            return {
                "sequence": seq, "session": session, "opcode": opcode,
                "size": size, "burst_complete": bool(burst), "offset": offset,
                "data": data,
            }

    @staticmethod
    def _entry(directory: str, raw: str):
        fields = raw.split("\t")
        if len(fields) < 2 or len(fields[0]) < 2:
            return None
        filename = fields[0][1:]
        lower = filename.lower()
        if not lower.endswith((".jpg", ".avi")) or ".thumbnail" in lower:
            return None
        try:
            size = int(fields[1])
            modified = int(fields[3]) if len(fields) > 3 else None
        except ValueError:
            return None
        return {
            "name": filename, "path": f"{directory}/{filename}",
            "type": "photo" if lower.endswith(".jpg") else "video",
            "size": size, "modified": modified,
        }

    def list_media(self):
        result = []
        for directory in MEDIA_DIRS:
            offset = 0
            encoded = directory.encode("utf-8")
            while True:
                self._request(3, encoded, offset=offset)
                response = self._response(3)
                if response["opcode"] == 129:  # NAK; EOF/no directory
                    break
                if response["opcode"] != 128:
                    raise MediaProtocolError("unexpected list response")
                rows = [row for row in response["data"].decode("utf-8", "replace").split("\0") if row]
                for row in rows:
                    entry = self._entry(directory, row)
                    if entry:
                        result.append(entry)
                if not rows:
                    break
                offset += len(rows)
        # Some firmware exposes the same file in more than one virtual folder.
        unique = {item["path"]: item for item in result}
        return sorted(unique.values(), key=lambda item: (item["modified"] or 0, item["name"]), reverse=True)

    def download(self, remote_path: str, destination: Path, expected_size: int):
        path = remote_path.encode("utf-8")
        self._request(4, path)
        opened = self._response(4)
        if opened["opcode"] != 128:
            raise FileNotFoundError(remote_path)
        session = opened["session"]
        received = 0
        with destination.open("wb") as output:
            while received < expected_size:
                self._request(16, session=session, offset=received, size=239)
                while True:
                    response = self._response(16)
                    if response["opcode"] == 129:  # EOF
                        return received
                    if response["opcode"] != 128 or response["offset"] != received:
                        raise MediaProtocolError("invalid file data response")
                    output.write(response["data"])
                    received += len(response["data"])
                    if received >= expected_size:
                        return received
                    if response["burst_complete"]:
                        break
        return received

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
