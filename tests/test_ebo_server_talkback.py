import base64
import os
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
import ebo_server


class _RunningProcess:
    @staticmethod
    def poll():
        return None


def bare_bridge():
    bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
    bridge._ctrl_r, bridge._ctrl_w = os.pipe()
    bridge._ctrl_lock = threading.Lock()
    bridge._speaker_ready = threading.Event()
    bridge._speaker_error = None
    bridge._speaker_state_lock = threading.Lock()
    bridge.paused = False
    bridge.proc = _RunningProcess()
    return bridge


class BridgeTalkbackTests(unittest.TestCase):
    def setUp(self):
        self.bridge = bare_bridge()

    def tearDown(self):
        os.close(self.bridge._ctrl_r)
        os.close(self.bridge._ctrl_w)

    def read_command(self):
        length = struct.unpack("<I", os.read(self.bridge._ctrl_r, 4))[0]
        return os.read(self.bridge._ctrl_r, length)

    def test_speaker_commands_use_expected_wire_kinds(self):
        self.bridge._send_command(2)
        self.assertEqual(self.read_command(), b"\x02")
        self.bridge.speaker_send_pcm(b"\x01\x02\x03\x04")
        self.assertEqual(self.read_command(), b"\x03\x01\x02\x03\x04")
        self.bridge.speaker_stop()
        self.assertEqual(self.read_command(), b"\x04")

    def test_pcm_validation(self):
        for invalid in (b"", b"\0", b"\0" * 1282):
            with self.subTest(length=len(invalid)):
                with self.assertRaises(ValueError):
                    self.bridge.speaker_send_pcm(invalid)

    def test_start_waits_for_native_ready(self):
        outcome = []
        thread = threading.Thread(
            target=lambda: outcome.append(self.bridge.speaker_start(1.0)))
        thread.start()
        self.assertEqual(self.read_command(), b"\x02")
        with self.bridge._speaker_state_lock:
            self.bridge._speaker_ready.set()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcome, [None])

    def test_start_timeout_sends_stop(self):
        with self.assertRaises(TimeoutError):
            self.bridge.speaker_start(0.01)
        self.assertEqual(self.read_command(), b"\x02")
        self.assertEqual(self.read_command(), b"\x04")

    def test_start_error_sends_stop(self):
        outcome = []
        def start():
            try:
                self.bridge.speaker_start(1.0)
            except RuntimeError as exc:
                outcome.append(str(exc))
        thread = threading.Thread(target=start)
        thread.start()
        self.assertEqual(self.read_command(), b"\x02")
        with self.bridge._speaker_state_lock:
            self.bridge._speaker_error = "-20009"
            self.bridge._speaker_ready.set()
        thread.join(1.0)
        self.assertEqual(self.read_command(), b"\x04")
        self.assertEqual(outcome, ["speaker start failed (-20009)"])


class BasicAuthTests(unittest.TestCase):
    def test_disabled_auth_allows(self):
        with mock.patch.object(ebo_server, "WEB_USER", None):
            self.assertTrue(ebo_server._valid_basic_auth(None))

    def test_valid_and_invalid_basic_auth(self):
        header = "Basic " + base64.b64encode(b"robot:secret").decode()
        with mock.patch.object(ebo_server, "WEB_USER", "robot"), \
             mock.patch.object(ebo_server, "WEB_PASS", "secret"):
            self.assertTrue(ebo_server._valid_basic_auth(header))
            self.assertFalse(ebo_server._valid_basic_auth("Basic !!!"))
            self.assertFalse(ebo_server._valid_basic_auth(
                "Basic " + base64.b64encode(b"robot:wrong").decode()))


class _FakeBridge:
    paused = False

    def __init__(self):
        self.started = 0
        self.stopped = 0
        self.frames = []

    def speaker_start(self, timeout):
        self.started += 1

    def speaker_send_pcm(self, pcm):
        self.frames.append(pcm)

    def speaker_stop(self):
        self.stopped += 1


class WebSocketTalkbackTests(unittest.TestCase):
    def test_disconnect_always_stops_speaker(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        fake = _FakeBridge()
        with mock.patch.object(ebo_server, "bridge", fake), \
             mock.patch.object(ebo_server, "WEB_USER", None):
            try:
                client = TestClient(ebo_server.app)
            except TypeError as exc:
                self.skipTest(f"incompatible TestClient: {exc}")
            with client.websocket_connect("/ws/talk") as websocket:
                self.assertEqual(websocket.receive_json()["type"], "ready")
                websocket.send_bytes(b"\x01\x02")
                deadline = time.monotonic() + 1.0
                while not fake.frames and time.monotonic() < deadline:
                    time.sleep(0.01)
            self.assertEqual(fake.started, 1)
            self.assertEqual(fake.frames, [b"\x01\x02"])
            self.assertEqual(fake.stopped, 1)


if __name__ == "__main__":
    unittest.main()
