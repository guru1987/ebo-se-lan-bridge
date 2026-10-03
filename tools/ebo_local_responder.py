#!/usr/bin/env python3
"""Minimal, LAN-only Kalay registration responder for an owned EBO SE.

This deliberately implements only the device registration exchange observed on
UDP/10240 and UDP/3478.  It is not an AV proxy and never opens a robot control
session.  Unknown packets and sources fail closed.
"""
from __future__ import annotations

import argparse
import ipaddress
import logging
import os
import selectors
import signal
import socket
import struct
from dataclasses import dataclass


LOGGER = logging.getLogger("ebo-local-responder")
CHARLIE = b"Charlie is the designer of P2P!!"
PORTS = (10240, 3478)
KEEPALIVE = b"\xcc\xaa\xab\xab"

_SWAPS = {
    2: (1, 0),
    4: (2, 3, 0, 1),
    8: (7, 4, 3, 2, 1, 6, 5, 0),
    16: (11, 9, 8, 15, 13, 10, 12, 14, 2, 1, 5, 0, 6, 4, 7, 3),
}


def _swap(data: bytes) -> bytes:
    order = _SWAPS.get(len(data))
    return bytes(data[index] for index in order) if order else data


def _rotate_left32(value: int, count: int) -> int:
    count %= 32
    return ((value << count) | (value >> (32 - count))) & 0xFFFFFFFF


def reverse_transcode(data: bytes) -> bytes:
    """Decode the reversible Kalay registration-packet transform."""
    output = bytearray()
    position = 0
    while len(data) - position >= 16:
        source = data[position:position + 16]
        rotated = b"".join(
            struct.pack("<I", _rotate_left32(struct.unpack_from("<I", source, offset)[0],
                                               offset + 3))
            for offset in range(0, 16, 4)
        )
        mixed = _swap(rotated)
        mixed = bytes(mixed[index] ^ CHARLIE[index] for index in range(16))
        output.extend(b"".join(
            struct.pack("<I", _rotate_left32(struct.unpack_from("<I", mixed, offset)[0],
                                               offset + 1))
            for offset in range(0, 16, 4)
        ))
        position += 16

    tail = _swap(data[position:])
    output.extend(tail[index] ^ CHARLIE[index] for index in range(len(tail)))
    return bytes(output)


def transcode(data: bytes) -> bytes:
    """Encode a Kalay registration packet; inverse of ``reverse_transcode``."""
    output = bytearray()
    position = 0
    while len(data) - position >= 16:
        source = data[position:position + 16]
        rotated = b"".join(
            struct.pack("<I", _rotate_left32(struct.unpack_from("<I", source, offset)[0],
                                               -offset - 1))
            for offset in range(0, 16, 4)
        )
        mixed = bytes(rotated[index] ^ CHARLIE[index] for index in range(16))
        mixed = _swap(mixed)
        output.extend(b"".join(
            struct.pack("<I", _rotate_left32(struct.unpack_from("<I", mixed, offset)[0],
                                               -offset - 3))
            for offset in range(0, 16, 4)
        ))
        position += 16

    tail = bytes(data[position + index] ^ CHARLIE[index]
                 for index in range(len(data) - position))
    output.extend(_swap(tail))
    return bytes(output)


@dataclass(frozen=True)
class Config:
    bind_ip: str
    robot_ip: str
    uid: bytes
    advertise_ip: str
    allowed_sources: frozenset[str]
    log_level: str = "INFO"

    @classmethod
    def from_environ(cls, environ: dict[str, str] | None = None) -> "Config":
        env = os.environ if environ is None else environ

        def ipv4(name: str) -> str:
            value = env.get(name, "").strip()
            try:
                parsed = ipaddress.ip_address(value)
            except ValueError as exc:
                raise ValueError(f"{name} must be an IPv4 address") from exc
            if parsed.version != 4:
                raise ValueError(f"{name} must be an IPv4 address")
            return str(parsed)

        bind_ip = ipv4("EBO_RESPONDER_BIND_IP")
        robot_ip = ipv4("EBO_RESPONDER_ROBOT_IP")
        advertise_ip = ipv4("EBO_RESPONDER_ADVERTISE_IP")
        uid_text = env.get("EBO_RESPONDER_UID", "").strip()
        try:
            uid = uid_text.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError("EBO_RESPONDER_UID must be ASCII") from exc
        if len(uid) != 20 or uid_text == "REPLACE_WITH_20_CHAR_UID":
            raise ValueError("EBO_RESPONDER_UID must be the owned robot's 20-character UID")

        source_text = env.get("EBO_RESPONDER_ALLOWED_SOURCES", robot_ip)
        allowed_sources = frozenset(
            str(ipaddress.ip_address(item.strip()))
            for item in source_text.split(",") if item.strip()
        )
        if not allowed_sources or any(ipaddress.ip_address(item).version != 4
                                      for item in allowed_sources):
            raise ValueError("EBO_RESPONDER_ALLOWED_SOURCES must contain IPv4 addresses")

        level = env.get("EBO_RESPONDER_LOG_LEVEL", "INFO").upper()
        if level not in {"DEBUG", "INFO", "WARNING", "ERROR"}:
            raise ValueError("EBO_RESPONDER_LOG_LEVEL is invalid")
        return cls(bind_ip, robot_ip, uid, advertise_ip, allowed_sources, level)


class RegistrationProtocol:
    """Pure packet handler used by both the UDP service and offline tests."""

    def __init__(self, config: Config):
        self.config = config
        self.counters = {"master": 0, "probe": 0, "login": 0, "register": 0,
                         "keepalive": 0, "dropped": 0}

    @staticmethod
    def _header(kind: bytes, payload_length: int, command: bytes) -> bytes:
        return kind + struct.pack("<I", payload_length) + command + b"\0" * 4

    def _source_endpoint(self, source_port: int) -> bytes:
        return (b"\x02\x00" + socket.inet_aton(self.config.robot_ip)
                + struct.pack("!H", source_port))

    def _master_response(self, source_port: int) -> bytes:
        endpoint = (b"\x02\x00" + struct.pack("!H", 3478)
                    + socket.inet_aton(self.config.advertise_ip) + b"\0" * 8)
        plain = (
            self._header(b"\x04\x02\x1d\x00", 110, b"\x08\x10\x83\x00")
            + self.config.uid + b"\0" * 16
            + self._source_endpoint(source_port) + b"\0" * 8
            + b"\x05\x00\x03\x00\x02\x00\x20\x00"
            + endpoint + endpoint
            + b"\x03\x00\x04\x00\x30\x00\x00\x00"
              b"\x01\x00\x06\x00\x18\xa2\x01\x00\x41\x01"
        )
        assert len(plain) == 126
        return transcode(plain)

    def _probe_response(self, request: bytes, source_port: int) -> bytes:
        token = request[20:22]
        plain = (
            self._header(b"\x04\x02\x1e\x00", 24, b"\x04\x80\x4f\x00")
            + self._source_endpoint(source_port) + b"\0" * 12 + token + b"\0\0"
        )
        assert len(plain) == 40
        return transcode(plain)

    def _login_response(self) -> bytes:
        plain = (
            self._header(b"\x04\x02\x1e\x00", 28, b"\x21\x01\x41\x00")
            + self.config.uid + b"\x03\0\0\0\x40\x01\0\0"
        )
        assert len(plain) == 44
        return transcode(plain)

    def _registration_response(self, request: bytes, source_port: int) -> bytes:
        # The final four bytes are the request's renewal nonce/timestamp.  The
        # broken experiment replayed an unrelated 78-byte packet here.  A real
        # master returns this 56-byte, request-bound acknowledgement.
        plain = (
            self._header(b"\x04\x02\x1e\x00", 40, b"\x23\x01\x41\x00")
            + self.config.uid + self._source_endpoint(source_port)
            + b"\0" * 8 + request[-4:]
        )
        assert len(plain) == 56
        return transcode(plain)

    def handle(self, local_port: int, packet: bytes,
               source: tuple[str, int]) -> bytes | None:
        source_ip, source_port = source
        if source_ip not in self.config.allowed_sources:
            self.counters["dropped"] += 1
            LOGGER.warning("drop port=%d source=%s reason=source", local_port, source_ip)
            return None
        if local_port not in PORTS:
            self.counters["dropped"] += 1
            return None
        if local_port == 3478 and packet == KEEPALIVE:
            self.counters["keepalive"] += 1
            return None

        try:
            plain = reverse_transcode(packet)
        except (IndexError, struct.error):
            self.counters["dropped"] += 1
            return None

        if local_port == 10240:
            if (len(plain) == 54 and plain[:12] ==
                    b"\x04\x02\x1d\x02\x26\0\0\0\x07\x10\x18\0"
                    and plain[16:36] == self.config.uid):
                self.counters["master"] += 1
                return self._master_response(source_port)
        elif len(plain) == 24 and plain[:12] == (
                b"\x04\x02\x1d\x02\x08\0\0\0\x03\x80\x3f\0"):
            self.counters["probe"] += 1
            return self._probe_response(plain, source_port)
        elif (len(plain) == 128 and plain[:12] ==
              b"\x04\x02\x1d\x02\x70\0\0\0\x20\x01\x14\0"
              and plain[16:36] == self.config.uid):
            self.counters["login"] += 1
            return self._login_response()
        elif (len(plain) == 48 and plain[:12] ==
              b"\x04\x02\x1d\x02\x20\0\0\0\x22\x01\x14\0"
              and plain[16:36] == self.config.uid):
            self.counters["register"] += 1
            return self._registration_response(plain, source_port)

        self.counters["dropped"] += 1
        LOGGER.warning("drop port=%d source=%s length=%d reason=packet",
                       local_port, source_ip, len(packet))
        return None


def serve(config: Config) -> None:
    protocol = RegistrationProtocol(config)
    selector = selectors.DefaultSelector()
    sockets: list[socket.socket] = []
    for port in PORTS:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((config.bind_ip, port))
        sock.setblocking(False)
        selector.register(sock, selectors.EVENT_READ, port)
        sockets.append(sock)

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    LOGGER.info("ready bind=%s ports=10240,3478 advertise=%s allowed=%s",
                config.bind_ip, config.advertise_ip,
                ",".join(sorted(config.allowed_sources)))
    try:
        while running:
            for key, _mask in selector.select(timeout=1.0):
                sock = key.fileobj
                packet, source = sock.recvfrom(2048)
                response = protocol.handle(key.data, packet, source)
                if response is not None:
                    sock.sendto(response, source)
                    LOGGER.info("reply port=%d source=%s:%d request=%d response=%d",
                                key.data, source[0], source[1], len(packet), len(response))
    finally:
        LOGGER.info("stopped counters=%s", protocol.counters)
        for sock in sockets:
            selector.unregister(sock)
            sock.close()
        selector.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-config", action="store_true",
                        help="validate environment and exit without binding sockets")
    args = parser.parse_args()
    try:
        config = Config.from_environ()
    except ValueError as exc:
        parser.error(str(exc))
    logging.basicConfig(level=getattr(logging, config.log_level),
                        format="%(asctime)s %(levelname)s %(name)s %(message)s")
    if args.check_config:
        LOGGER.info("configuration valid")
        return 0
    serve(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
