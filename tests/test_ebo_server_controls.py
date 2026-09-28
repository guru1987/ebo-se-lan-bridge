import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))

import ebo_server
from ebo_mavlink import FrameBuilder, MAVLinkV1Stream


def state_bridge():
    bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
    bridge.parameters = {}
    bridge.parameter_meta = {}
    bridge.telemetry = {}
    bridge.timed_tasks = {1: {}, 2: {}, 3: {}}
    bridge._task_list_counts = {1: None, 2: None, 3: None}
    bridge.last_task_ack = None
    bridge.file_server_info = None
    bridge._mav_stream = MAVLinkV1Stream()
    bridge._state_condition = threading.Condition()
    bridge.on_status = None
    bridge.on_state = None
    bridge.battery = -1
    bridge.charge = -1
    return bridge


class StateDecoderTests(unittest.TestCase):
    def test_motor_frame_preserves_positive_forward_api_contract(self):
        frame = MAVLinkV1Stream().feed(ebo_server.motor_frame(ly=0.25, rx=-0.5))[0]
        self.assertEqual(frame.message_id, 202)
        lx, ly, rx, ry = struct.unpack_from("<4f", frame.payload)
        self.assertEqual((lx, ly, rx, ry, frame.payload[16]), (0.0, -0.25, -0.5, 0.0, 0))

    def test_fragmented_parameter_echo_updates_state_and_callback(self):
        bridge = state_bridge()
        changes = []
        bridge.on_state = lambda *change: changes.append(change)
        payload = (struct.pack("<fHH", 61.0, 2, 1) +
                   b"audio-volume".ljust(32, b"\0") + b"\x06")
        frame = FrameBuilder().frame(228, payload)
        bridge._parse_status(frame[:7])
        self.assertEqual(bridge.parameters, {})
        bridge._parse_status(frame[7:])
        self.assertEqual(bridge.parameters["audio-volume"], 61.0)
        self.assertEqual(changes[-1], ("parameter", "audio-volume", 61.0))

    def test_battery_details_and_legacy_status_callback(self):
        bridge = state_bridge()
        status = []
        bridge.on_status = lambda battery, charge: status.append((battery, charge))
        frame = FrameBuilder().frame(207, struct.pack("<4fBB", 8.1, -0.1, 8.0, 8.4, 91, 1))
        bridge._parse_status(frame)
        self.assertEqual((bridge.battery, bridge.charge), (91, 1))
        self.assertEqual(status, [(91, 1)])
        self.assertAlmostEqual(bridge.telemetry["battery"]["measured_voltage"], 8.1, places=5)

    def test_zero_count_timed_task_item_is_an_empty_list_sentinel(self):
        bridge = state_bridge()
        payload = struct.pack("<7i9B", 0, 0, 0, 0, 0, 0, 0,
                              2, 0, 0, 0, 0, 0, 0, 0, 0)
        bridge._parse_status(FrameBuilder().frame(154, payload))
        self.assertEqual(bridge.timed_tasks[2], {})
        self.assertEqual(bridge._task_list_counts[2], 0)


class StatefulParameterTests(unittest.TestCase):
    def test_write_requires_and_accepts_matching_robot_echo(self):
        bridge = state_bridge()
        builder = FrameBuilder()

        def echo(_request):
            payload = (struct.pack("<fHH", 75.0, 1, 0) +
                       b"audio-volume".ljust(32, b"\0") + b"\x06")
            bridge._parse_status(builder.frame(228, payload))

        bridge.send_rdt = echo
        result = bridge.set_parameter("audio", "volume", 75)
        self.assertTrue(result["acknowledged"])
        self.assertEqual(result["value"], 75.0)

    def test_unknown_and_out_of_range_parameters_are_rejected(self):
        bridge = state_bridge()
        bridge.send_rdt = lambda _: self.fail("invalid writes must not be sent")
        with self.assertRaises(ValueError):
            bridge.set_parameter("not", "real", 1)
        with self.assertRaises(ValueError):
            bridge.set_parameter("audio", "volume", 101)
        with self.assertRaises(ValueError):
            bridge.set_parameter("security_fixed", "sensitivity", 3)
        with self.assertRaises(ValueError):
            bridge.set_parameter("security_fixed", "notify_interval", 61)

    def test_day_night_uses_rola_numeric_modes(self):
        self.assertEqual(ebo_server.PARAMETERS["video-night_vision"],
                         {"type": 6, "min": 0, "max": 2})
        self.assertEqual(ebo_server.PARAM_TOGGLES["day"], ("video", "night_vision", 1.0))
        self.assertEqual(ebo_server.PARAM_TOGGLES["night_on"], ("video", "night_vision", 2.0))


class ControlAPITests(unittest.TestCase):
    def test_move_rejects_axes_outside_normalized_range(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        with mock.patch.object(ebo_server, "WEB_USER", None):
            try:
                client = TestClient(ebo_server.app)
            except TypeError as exc:
                self.skipTest(f"incompatible TestClient: {exc}")
            response = client.post("/move", json={"ly": 50, "rx": 0, "duration": 0.2})
        self.assertEqual(response.status_code, 422)

    def test_parameter_put_returns_echoed_robot_state(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        bridge = state_bridge()
        builder = FrameBuilder()

        def echo(_request):
            payload = (struct.pack("<fHH", 44.0, 1, 0) +
                       b"audio-volume".ljust(32, b"\0") + b"\x06")
            bridge._parse_status(builder.frame(228, payload))

        bridge.send_rdt = echo
        with mock.patch.object(ebo_server, "bridge", bridge), \
             mock.patch.object(ebo_server, "WEB_USER", None):
            try:
                client = TestClient(ebo_server.app)
            except TypeError as exc:
                self.skipTest(f"incompatible TestClient: {exc}")
            response = client.put("/parameter/audio/volume", json={"value": 44})
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["acknowledged"])


class VideoAndAudioPublicationTests(unittest.TestCase):
    def video_bridge(self):
        bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
        bridge.frame_count = 20
        bridge.video_enabled = False
        bridge._published_frame_count = 10
        bridge._last_published_frame = 0.0
        bridge._video_start_publish_count = 10
        bridge._ff_primed = True
        bridge._state_condition = threading.Condition()
        return bridge

    def test_video_start_waits_for_a_new_published_frame(self):
        bridge = self.video_bridge()
        sent = threading.Event()

        def send_ioctl(io_type, data):
            self.assertEqual((io_type, data), (0x01ff, bytes(8)))
            sent.set()

        bridge.send_ioctl = send_ioctl

        def publish():
            self.assertTrue(sent.wait(0.5))
            with bridge._state_condition:
                bridge._ff_primed = True
                bridge._published_frame_count += 1
                bridge._last_published_frame = time.time()
                bridge._state_condition.notify_all()

        worker = threading.Thread(target=publish)
        worker.start()
        self.assertTrue(bridge.set_video(True, timeout=0.5))
        worker.join()
        self.assertTrue(bridge.video_ready())
        self.assertTrue(bridge._ff_primed)

    def test_video_start_times_out_without_a_publishable_frame(self):
        bridge = self.video_bridge()
        bridge.send_ioctl = lambda *_: None
        self.assertFalse(bridge.set_video(True, timeout=0.01))
        self.assertFalse(bridge.video_ready())

    def test_video_stop_is_immediate_and_not_ready(self):
        bridge = self.video_bridge()
        calls = []
        bridge.send_ioctl = lambda *args: calls.append(args)
        self.assertFalse(bridge.set_video(False))
        self.assertEqual(calls, [(0x02ff, bytes(8))])
        self.assertFalse(bridge.video_ready())

    def test_audio_only_ffmpeg_publishes_separate_rtsp_path(self):
        bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
        bridge._ao_r = 17
        process = mock.Mock(pid=123)
        with mock.patch.object(ebo_server.subprocess, "Popen", return_value=process) as popen:
            bridge._start_audio_ffmpeg()
        command = popen.call_args.args[0]
        self.assertIn("rtsp://127.0.0.1:8554/ebo_audio", command)
        self.assertIn("-vn", command)
        self.assertEqual(popen.call_args.kwargs["pass_fds"], (17,))


if __name__ == "__main__":
    unittest.main()
