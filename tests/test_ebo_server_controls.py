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

    def test_special_motion_commands_match_rola_command_mini_payloads(self):
        expected = {
            "big_circle": 40115, "rotate": 40115, "donut": 40115,
            "straight_line": 40116, "rush_forward": 40116, "rocket": 40116,
            "swing": 40129, "shake": 40129,
            "spin": 40164, "race": 40165, "excited": 40166,
            "relaxed": 40167, "little_monster": 40168,
        }
        self.assertEqual(ebo_server.SPECIAL_MOTION_COMMANDS, expected)
        for name, command in expected.items():
            frame = MAVLinkV1Stream().feed(ebo_server.command_frame(command))[0]
            self.assertEqual(frame.message_id, 200, name)
            self.assertEqual(frame.payload, struct.pack("<HBB", command, 255, 1), name)

    def test_special_motion_action_sends_one_frame(self):
        fake_bridge = mock.Mock()
        with mock.patch.object(ebo_server, "bridge", fake_bridge):
            self.assertTrue(ebo_server.do_action("spin"))
        fake_bridge.send_rdt.assert_called_once_with(ebo_server.command_frame(40164))

    def test_unknown_special_motion_is_not_sent(self):
        fake_bridge = mock.Mock()
        with mock.patch.object(ebo_server, "bridge", fake_bridge):
            self.assertFalse(ebo_server.do_action("backflip"))
        fake_bridge.send_rdt.assert_not_called()

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


class TrickGateTests(unittest.TestCase):
    """GET /actions and the charging/depth gate on POST /action/<trick>; nothing real is sent."""

    def client(self, charge, guard=None):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        self.fake = mock.Mock()
        self.fake.charge, self.fake.battery = charge, 80
        patches = [mock.patch.object(ebo_server, "bridge", self.fake),
                   mock.patch.object(ebo_server, "guard", guard),
                   mock.patch.object(ebo_server, "WEB_USER", None)]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        try:
            return TestClient(ebo_server.app)
        except TypeError as exc:
            self.skipTest(f"incompatible TestClient: {exc}")

    def test_actions_lists_toggles_and_tricks(self):
        r = self.client(1).get("/actions")
        self.assertEqual(r.status_code, 200)
        acts = {a["name"]: a for a in r.json()["actions"]}
        for n in ("dock", "undock", "eyes_on", "sleep", "wake", "autoreturn_on"):
            self.assertFalse(acts[n]["moves"], n)
        for n, c in ebo_server.SPECIAL_MOTION_COMMANDS.items():
            self.assertTrue(acts[n]["moves"], n)
            self.assertEqual(acts[n]["command"], c)
        self.assertEqual(set(acts), set(ebo_server.BASIC_ACTIONS) | set(ebo_server.PARAM_TOGGLES)
                         | set(ebo_server.SPECIAL_MOTION_COMMANDS))
        self.fake.send_rdt.assert_not_called()

    def test_trick_refused_while_charging(self):
        r = self.client(1).post("/action/donut").json()
        self.assertFalse(r["ok"])
        self.assertTrue(r["refused"])
        self.assertIn("charging", r["reason"])
        self.fake.send_rdt.assert_not_called()

    def test_trick_refused_when_charge_unknown(self):
        r = self.client(-1).post("/action/spin").json()
        self.assertTrue(r["refused"])
        self.fake.send_rdt.assert_not_called()

    def test_trick_force_sends_exactly_one_frame(self):
        r = self.client(1).post("/action/rocket?force=1").json()
        self.assertTrue(r["ok"])
        self.fake.send_rdt.assert_called_once_with(ebo_server.command_frame(40116))

    def test_trick_unplugged_without_guard_sends_one_frame(self):
        r = self.client(0).post("/action/little_monster").json()
        self.assertTrue(r["ok"])
        self.fake.send_rdt.assert_called_once_with(ebo_server.command_frame(40168))

    def test_trick_refused_by_depth_guard(self):
        g = mock.Mock()
        g.wait_ready.return_value = (False, {"reason": "too close", "free_ahead_m": 0.1})
        r = self.client(0, guard=g).post("/action/swing").json()
        self.assertTrue(r["refused"])
        self.assertIn("depth guard", r["reason"])
        self.fake.send_rdt.assert_not_called()

    def test_trick_allowed_by_depth_guard(self):
        g = mock.Mock()
        g.wait_ready.return_value = (True, {"free_ahead_m": 1.0})
        r = self.client(0, guard=g).post("/action/race").json()
        self.assertTrue(r["ok"])
        self.fake.send_rdt.assert_called_once_with(ebo_server.command_frame(40165))

    def test_unknown_action_still_unknown(self):
        r = self.client(0).post("/action/backflip").json()
        self.assertEqual(r, {"ok": False, "err": "unknown"})
        self.fake.send_rdt.assert_not_called()


class DockGuardRailTests(unittest.TestCase):
    """While on the dock only a straight forward /move or /drive is allowed; nothing real is sent."""

    def client(self, charge):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        self.fake = mock.Mock()
        self.fake.charge, self.fake.battery = charge, 80
        patches = [mock.patch.object(ebo_server, "bridge", self.fake),
                   mock.patch.object(ebo_server, "guard", None),
                   mock.patch.object(ebo_server, "WEB_USER", None),
                   mock.patch.object(ebo_server.time, "sleep", lambda _s: None)]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        ebo_server._dock_log_last.clear()
        try:
            return TestClient(ebo_server.app)
        except TypeError as exc:
            self.skipTest(f"incompatible TestClient: {exc}")

    def refused(self, c, path, ly, rx):
        r = c.post(path, json={"ly": ly, "rx": rx, "duration": 0.2}).json()
        self.assertFalse(r["ok"])
        self.assertTrue(r["refused"])
        self.assertTrue(r["dock_guard"])
        self.assertIn("on the dock: only straight forward allowed — drive off first", r["reason"].replace(
            "charge state unknown, assuming on the dock", "on the dock"))
        self.fake.send_rdt.assert_not_called()
        return r

    def test_docked_refuses_turn_and_back_on_move_and_drive(self):
        c = self.client(1)
        for path in ("/move", "/drive"):
            for ly, rx in ((0, 0.5), (0, -0.5), (0.5, 0.3), (0.5, -0.3), (-0.5, 0), (-0.5, 0.4)):
                self.refused(c, path, ly, rx)

    def test_docked_refusal_is_logged(self):
        c = self.client(1)
        with mock.patch("builtins.print") as pr:
            self.refused(c, "/move", -0.5, 0)
        self.assertTrue(any("[dock] REFUSED" in str(a[0][0]) for a in pr.call_args_list))

    def test_docked_straight_forward_is_allowed(self):
        c = self.client(1)
        self.assertTrue(c.post("/move", json={"ly": 0.5, "rx": 0, "duration": 0.2}).json()["ok"])
        sent = [call.args[0] for call in self.fake.send_rdt.call_args_list]
        self.assertIn(ebo_server.motor_frame(ly=0.5, rx=0.0), sent)
        self.fake.send_rdt.reset_mock()
        self.assertTrue(c.post("/drive", json={"ly": 0.5, "rx": 0}).json()["ok"])
        self.fake.send_rdt.assert_called_once_with(ebo_server.motor_frame(ly=0.5, rx=0.0))

    def test_unknown_charge_refuses_turn_and_back_allows_forward(self):
        c = self.client(-1)
        r = self.refused(c, "/move", 0, 0.5)
        self.assertEqual(r["charge"], -1)
        self.refused(c, "/drive", -0.5, 0)
        self.assertTrue(c.post("/drive", json={"ly": 0.5, "rx": 0}).json()["ok"])

    def test_unplugged_allows_everything(self):
        c = self.client(0)
        for path in ("/move", "/drive"):
            for ly, rx in ((0, 0.5), (-0.5, 0), (0.5, 0.3)):
                self.fake.send_rdt.reset_mock()
                self.assertTrue(c.post(path, json={"ly": ly, "rx": rx, "duration": 0.2}).json()["ok"])
                self.fake.send_rdt.assert_called()

    def test_dock_and_undock_actions_unchanged(self):
        c = self.client(1)
        self.assertEqual(c.post("/action/undock").json(), {"ok": True, "action": "undock"})
        self.assertEqual(c.post("/action/dock").json(), {"ok": True, "action": "dock"})


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

    def test_video_quality_uses_rola_vendor_payload(self):
        bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
        bridge.video_quality = None
        bridge.video_quality_updated = 0.0
        calls = []
        bridge.send_ioctl = lambda *args: calls.append(args)

        result = bridge.set_video_quality("Medium")

        self.assertEqual(calls, [(ebo_server.VIDEO_QUALITY_IOCTL,
                                  struct.pack("<III", 2, 1500, 1))])
        self.assertEqual(result["level"], "medium")
        self.assertEqual(result["target_resolution"], "848x480")
        self.assertEqual(result["target_bitrate_kbps"], 1500)
        self.assertEqual(bridge.video_quality, "medium")

    def test_video_quality_rejects_unknown_level_without_sending(self):
        bridge = ebo_server.Bridge.__new__(ebo_server.Bridge)
        bridge.send_ioctl = lambda *_: self.fail("invalid quality must not be sent")
        with self.assertRaisesRegex(ValueError, "auto, high, medium, or low"):
            bridge.set_video_quality("ultra")


if __name__ == "__main__":
    unittest.main()
