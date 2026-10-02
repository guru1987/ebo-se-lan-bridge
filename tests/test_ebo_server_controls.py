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

    def test_dock_and_undock_actions_allowed_on_the_dock(self):
        c = self.client(1)
        with mock.patch.object(ebo_server, "UNDOCK_CHARGE_POLL_S", 0.0):
            r = c.post("/action/undock").json()
        self.assertTrue(r["ok"])
        self.assertEqual(r["action"], "undock")
        self.assertEqual(c.post("/action/dock").json(),
                         {"ok": True, "action": "dock", "dockwatch": {"watching": False}})


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


NO_READING = {"reason": "depth not running", "no_fresh_reading": True, "depth_running": False}


def good(free=1.0):
    return {"free_ahead_m": free, "depth_running": True, "depth_age_s": 0.05}


def too_close(free=0.1):
    return {"free_ahead_m": free, "depth_running": True, "reason": "too close"}


class ScriptedGuard(ebo_server.DepthGuard):
    """Real DepthGuard (thread, verdicts, wait/wake logic) with check() scripted: script(t) -> info
    (t = seconds since the first check) where info without "reason" means ok."""

    def __init__(self, script):
        self.script, self.t0 = script, None
        super().__init__("http://127.0.0.1:9/none", "", None, None)

    def check(self):
        if self.t0 is None:
            self.t0 = time.monotonic()
        info = {"stop_free_m": 0.25, **self.script(time.monotonic() - self.t0)}
        return "reason" not in info, info


class FakeVideoBridge:
    """Bridge stand-in: records motor frames; video starts paused; set_video(True) resumes it."""

    def __init__(self, charge=1, video=False):
        self.charge, self.battery, self.paused = charge, 80, False
        self.video = video
        self.frames = []
        self.set_video_calls = 0
        self.lock = threading.Lock()

    def send_rdt(self, frame):
        with self.lock:
            self.frames.append(frame)

    def video_ready(self):
        return self.video

    def set_video(self, enabled, timeout=12):
        self.set_video_calls += 1
        self.video_resumed_at = time.monotonic()
        self.video = enabled
        return enabled

    def count(self, frame):
        with self.lock:
            return self.frames.count(frame)


class GuardWakeTests(unittest.TestCase):
    """No fresh depth reading (video paused by ebo-eye): wake the stream, wait, then decide."""

    def setUp(self):
        self.fake = FakeVideoBridge()
        for p in (mock.patch.object(ebo_server, "bridge", self.fake),
                  mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", ""),
                  mock.patch.object(ebo_server, "WEB_USER", None)):
            p.start()
            self.addCleanup(p.stop)

    def resumed_script(self, up_after=0.3, reading_after=0.5, reading=good):
        """No reading until the video is resumed; decoder up up_after s later; reading after that."""
        def script(_t):
            t = getattr(self.fake, "video_resumed_at", None)
            if t is None:
                return NO_READING
            dt = time.monotonic() - t
            if dt < up_after:
                return NO_READING
            if dt < reading_after:
                return {"reason": "no depth reading yet", "no_fresh_reading": True, "depth_running": True}
            return reading()
        return script

    def test_wakes_video_and_waits_for_a_fresh_reading(self):
        g = ScriptedGuard(self.resumed_script())
        ok, info = g.wait_ready(0.1, wake_wait=1.0)   # the start window alone (0.1 s) is too short
        self.assertTrue(ok, info)
        self.assertTrue(info["woke"])
        self.assertEqual(info["free_ahead_m"], 1.0)
        self.assertEqual(self.fake.set_video_calls, 1)

    def test_still_refused_when_no_reading_arrives(self):
        g = ScriptedGuard(lambda _t: {"reason": "no depth reading yet", "no_fresh_reading": True,
                                      "depth_running": True})
        t0 = time.monotonic()
        ok, info = g.wait_ready(0.1, wake_wait=0.3)
        waited = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertTrue(info["woke"])
        self.assertIn("no depth reading", info["reason"])
        self.assertGreater(waited, 0.3)
        self.assertLess(waited, 1.0)

    def test_wake_wait_is_capped_when_the_stream_never_comes_up(self):
        g = ScriptedGuard(lambda _t: NO_READING)
        with mock.patch.object(ebo_server, "GUARD_WAKE_MAX_S", 0.4):
            t0 = time.monotonic()
            ok, info = g.wait_ready(0.1, wake_wait=5.0)
        self.assertFalse(ok)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_fresh_but_too_close_is_not_a_wake(self):
        g = ScriptedGuard(lambda _t: too_close())
        ok, info = g.wait_ready(0.2)
        self.assertFalse(ok)
        self.assertNotIn("woke", info)
        self.assertEqual(self.fake.set_video_calls, 0)

    def test_after_wake_waits_for_a_reading_with_floor_scale(self):
        """Live 2026-10-02: the first readings after a wake had no floor scale yet -> wait for one."""
        state = {}

        def script(_t):
            if self.fake.set_video_calls == 0:
                return NO_READING
            state.setdefault("t", time.monotonic())
            dt = time.monotonic() - state["t"]
            if dt < 0.1:
                return {"reason": "no depth reading yet", "no_fresh_reading": True, "depth_running": True}
            if dt < 0.6:   # fresh, but no floor scale (camera settling)
                return {"reason": "distance ahead unknown (no floor scale)", "no_scale": True,
                        "free_ahead_m": None, "depth_running": True}
            return good(0.45)
        g = ScriptedGuard(script)
        ok, info = g.wait_ready(0.1, wake_wait=0.2, scale_wait=1.5)   # stream-up wait alone too short
        self.assertTrue(ok, info)
        self.assertTrue(info["woke"])
        self.assertEqual(info["free_ahead_m"], 0.45)

    def test_after_wake_refused_when_no_scaled_reading_arrives(self):
        def script(_t):
            if self.fake.set_video_calls == 0:
                return NO_READING
            return {"reason": "distance ahead unknown (no floor scale)", "no_scale": True,
                    "free_ahead_m": None, "depth_running": True}
        g = ScriptedGuard(script)
        t0 = time.monotonic()
        ok, info = g.wait_ready(0.1, wake_wait=0.1, scale_wait=0.5)
        waited = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertTrue(info["woke"])
        self.assertIn("no floor scale", info["reason"])
        self.assertGreater(waited, 0.45)
        self.assertLess(waited, 1.2)

    def test_after_wake_first_scaled_reading_decides_at_once(self):
        def script(_t):
            if self.fake.set_video_calls == 0:
                return NO_READING
            return too_close(0.12)
        g = ScriptedGuard(script)
        t0 = time.monotonic()
        ok, info = g.wait_ready(0.1, wake_wait=2.0, scale_wait=3.0)
        self.assertFalse(ok)
        self.assertEqual(info["free_ahead_m"], 0.12)
        self.assertLess(time.monotonic() - t0, 0.8)

    def test_wake_asks_ebo_eye_first(self):
        g = ScriptedGuard(lambda _t: NO_READING)
        keep = mock.Mock(side_effect=lambda: setattr(self.fake, "video", True))
        with mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", "http://eye/video_keep"), \
                mock.patch.object(g, "_video_keep", keep):
            self.assertTrue(g.wake())
            for _ in range(100):
                if not g._waking:
                    break
                time.sleep(0.01)
        keep.assert_called_once()
        self.assertEqual(self.fake.set_video_calls, 0)   # eye already resumed it

    def test_forward_move_wakes_then_moves(self):
        g = ScriptedGuard(self.resumed_script())
        with mock.patch.object(ebo_server, "guard", g), \
                mock.patch.object(ebo_server, "GUARD_START_WAIT_S", 0.1), \
                mock.patch.object(ebo_server, "GUARD_WAKE_WAIT_S", 1.0):
            r = ebo_server.do_move(0.5, 0.0, 0.2)
        self.assertTrue(r["ok"], r)
        self.assertTrue(r["guard"]["woke"])
        self.assertGreaterEqual(self.fake.count(ebo_server.motor_frame(ly=0.5)), 1)

    def test_forward_move_refused_without_reading_sends_no_motion(self):
        g = ScriptedGuard(lambda _t: NO_READING)
        with mock.patch.object(ebo_server, "guard", g), \
                mock.patch.object(ebo_server, "GUARD_START_WAIT_S", 0.1), \
                mock.patch.object(ebo_server, "GUARD_WAKE_MAX_S", 0.3):
            r = ebo_server.do_move(0.5, 0.0, 0.2)
        self.assertFalse(r["ok"])
        self.assertEqual(r["phase"], "start")
        self.assertTrue(r["woke"])
        self.assertEqual(self.fake.count(ebo_server.motor_frame(ly=0.5)), 0)

    def test_drive_wakes_then_moves(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        g = ScriptedGuard(self.resumed_script())
        with mock.patch.object(ebo_server, "guard", g), \
                mock.patch.object(ebo_server, "GUARD_WAKE_WAIT_S", 1.0):
            r = TestClient(ebo_server.app).post("/drive", json={"ly": 0.5, "rx": 0}).json()
        self.assertTrue(r["ok"], r)
        self.assertEqual(self.fake.set_video_calls, 1)
        self.assertEqual(self.fake.count(ebo_server.motor_frame(ly=0.5)), 1)


class UndockTests(unittest.TestCase):
    """Undock = a straight forward move (EBO_UNDOCK_LY x EBO_UNDOCK_S) through the forward guard."""

    def setUp(self):
        self.fake = FakeVideoBridge(charge=1, video=True)
        orig = self.fake.send_rdt

        def send(frame):   # the robot leaves the dock once it is driven off and stopped
            orig(frame)
            if frame == ebo_server.motor_frame() and self.fake.count(ebo_server.motor_frame(ly=0.6)) >= 1:
                self.fake.charge = 0
        self.fake.send_rdt = send
        for p in (mock.patch.object(ebo_server, "bridge", self.fake),
                  mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", ""),
                  mock.patch.object(ebo_server, "WEB_USER", None)):
            p.start()
            self.addCleanup(p.stop)

    def test_defaults_are_a_real_move(self):
        self.assertEqual((ebo_server.UNDOCK_LY, ebo_server.UNDOCK_S), (0.6, 2.0))

    def test_undock_drives_full_duration_and_reports_charge(self):
        with mock.patch.object(ebo_server, "guard", None), \
                mock.patch.object(ebo_server.time, "sleep", lambda _s: None):
            r = ebo_server.do_undock()
        self.assertEqual(self.fake.count(ebo_server.motor_frame(ly=0.6)), 40)   # 2.0 s at 20 Hz
        self.assertEqual(self.fake.frames[-1], ebo_server.motor_frame())
        self.assertTrue(r["ok"])
        self.assertEqual((r["charge_before"], r["charge_after"], r["left_dock"]), (1, 0, True))
        self.assertEqual((r["ly"], r["duration_s"]), (0.6, 2.0))

    def test_undock_endpoint_allowed_while_docked(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        g = ScriptedGuard(lambda _t: good(0.9))
        with mock.patch.object(ebo_server, "guard", g), mock.patch.object(ebo_server, "UNDOCK_S", 0.3):
            r = TestClient(ebo_server.app).post("/action/undock").json()
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["action"], "undock")
        self.assertTrue(r["left_dock"])
        self.assertEqual(r["guard"]["free_ahead_m_start"], 0.9)
        self.assertEqual(self.fake.count(ebo_server.motor_frame(ly=0.6)), 6)

    def test_undock_refused_by_guard_does_not_move(self):
        g = ScriptedGuard(lambda _t: too_close(0.12))
        with mock.patch.object(ebo_server, "guard", g), mock.patch.object(ebo_server, "GUARD_START_WAIT_S", 0.2):
            r = ebo_server.do_undock()
        self.assertFalse(r["ok"])
        self.assertTrue(r["stopped_by_guard"])
        self.assertEqual(r["phase"], "start")
        self.assertEqual(r["free_ahead_m"], 0.12)
        self.assertEqual((r["charge_after"], r["left_dock"]), (1, False))
        self.assertEqual(self.fake.count(ebo_server.motor_frame(ly=0.6)), 0)

    def test_undock_stopped_mid_way_by_guard(self):
        g = ScriptedGuard(lambda t: good(0.6) if t < 0.3 else too_close(0.2))
        with mock.patch.object(ebo_server, "guard", g), mock.patch.object(ebo_server, "UNDOCK_S", 2.0), \
                mock.patch.object(ebo_server, "UNDOCK_CHARGE_POLL_S", 0.2):
            r = ebo_server.do_undock()
        self.assertFalse(r["ok"])
        self.assertEqual(r["phase"], "moving")
        self.assertLess(r["moved_s"], 1.5)
        self.assertLess(self.fake.count(ebo_server.motor_frame(ly=0.6)), 30)
        self.assertEqual(self.fake.frames[-1], ebo_server.motor_frame())

    def test_undock_waits_for_reading_when_video_paused(self):
        self.fake.video = False
        state = {}

        def script(_t):
            if self.fake.set_video_calls == 0:
                return NO_READING
            state.setdefault("t", time.monotonic())
            return good(1.2) if time.monotonic() - state["t"] > 0.3 else NO_READING
        g = ScriptedGuard(script)
        with mock.patch.object(ebo_server, "guard", g), mock.patch.object(ebo_server, "UNDOCK_S", 0.3), \
                mock.patch.object(ebo_server, "GUARD_START_WAIT_S", 0.1):
            r = ebo_server.do_undock()
        self.assertTrue(r["ok"], r)
        self.assertTrue(r["guard"]["woke"])
        self.assertTrue(r["left_dock"])


class ScriptedDockWatch(ebo_server.DockWatch):
    """Real DockWatch thread/logic; fetch() plays a list of wide-corridor free distances, one fresh
    reading (new seq) per poll; after the list it keeps returning the last one (no new reading)."""

    def __init__(self, values, on_poll=None):
        super().__init__("http://127.0.0.1:9/none", None, None)
        self.values, self.i, self.on_poll = list(values), 0, on_poll

    def fetch(self):
        if self.on_poll:
            self.on_poll(self.i)
        i = min(self.i, len(self.values) - 1)
        self.i += 1
        v = self.values[i]
        return {"seq": i, "frame_t": time.time(), "free_ahead_m": v, "free_ahead_kind": "obstacle",
                "free_ahead_wide_m": v, "free_ahead_wide_kind": "obstacle", "wide_half_width_m": 0.17}


class DockWatchTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeVideoBridge(charge=0, video=True)
        for p in (mock.patch.object(ebo_server, "bridge", self.fake),
                  mock.patch.object(ebo_server, "guard", None),
                  mock.patch.object(ebo_server, "DOCKWATCH_POLL_S", 0.005),
                  mock.patch.object(ebo_server, "DOCKWATCH_CONFIRM_S", 0.15),
                  mock.patch.object(ebo_server, "DOCKWATCH_TIMEOUT_S", 2.0),
                  mock.patch.object(ebo_server, "DOCKWATCH_RECOVER", False),
                  mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", ""),
                  mock.patch.object(ebo_server, "WEB_USER", None)):
            p.start()
            self.addCleanup(p.stop)

    def run_watch(self, values, on_poll=None, timeout=3.0):
        w = ScriptedDockWatch(values, on_poll)
        self.watch = w
        with mock.patch.object(ebo_server, "dockwatch", w):
            w.start()
            t0 = time.monotonic()
            while w.status()["active"] and time.monotonic() - t0 < timeout:
                time.sleep(0.01)
        return w

    def stops(self):
        return self.fake.count(ebo_server.motor_frame())

    def test_approach_toward_obstacle_stops(self):
        w = self.run_watch([0.6, 0.5, 0.42, 0.35, 0.29, 0.24, 0.2])
        ev = w.status()["last_event"]
        self.assertEqual(ev["kind"], "stopped", ev)
        self.assertEqual(ev["free_m"], 0.24)
        self.assertEqual(ev["message"], "docking stopped: obstacle 0.24 m ahead")
        self.assertGreaterEqual(self.stops(), 5)
        self.assertEqual(w.take_notice(), "docking stopped: obstacle 0.24 m ahead")
        self.assertIsNone(w.take_notice())   # once

    def test_still_closing_after_stop_resends(self):
        w = self.run_watch([0.4, 0.33, 0.28, 0.24, 0.21, 0.18, 0.15])
        ev = w.status()["last_event"]
        self.assertEqual(ev["kind"], "stopped")
        self.assertTrue(ev.get("still_closing_after_stop"), ev)
        self.assertGreaterEqual(ev["resends"], 1)
        self.assertGreaterEqual(self.stops(), 10)

    def test_reversing_with_growing_distance_does_not_stop(self):
        w = self.run_watch([0.12, 0.14, 0.17, 0.2, 0.22, 0.25, 0.3], on_poll=self.dock_after(10))
        self.assertEqual(w.status()["last_event"]["kind"], "docked")
        self.assertEqual(self.stops(), 0)

    def test_stable_close_distance_does_not_stop(self):
        w = self.run_watch([0.2, 0.2, 0.21, 0.2, 0.2, 0.21, 0.2], on_poll=self.dock_after(10))
        self.assertEqual(w.status()["last_event"]["kind"], "docked")
        self.assertEqual(self.stops(), 0)

    def test_turning_in_place_does_not_stop(self):
        # the view sweeps: distances jump around rather than closing steadily
        w = self.run_watch([1.2, 0.22, 0.9, 0.18, 0.5, 0.2, 1.5, 0.15, 0.7], on_poll=self.dock_after(12))
        self.assertEqual(w.status()["last_event"]["kind"], "docked")
        self.assertEqual(self.stops(), 0)

    def test_far_approach_does_not_stop(self):
        w = self.run_watch([1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3], on_poll=self.dock_after(12))
        self.assertEqual(self.stops(), 0)

    def test_charging_ends_the_watch(self):
        self.fake.charge = 1
        w = self.run_watch([0.6, 0.4, 0.2])
        ev = w.status()["last_event"]
        self.assertEqual(ev["kind"], "docked")
        self.assertFalse(w.status()["active"])
        self.assertEqual(self.stops(), 0)

    def test_timeout_ends_the_watch(self):
        with mock.patch.object(ebo_server, "DOCKWATCH_TIMEOUT_S", 0.2):
            w = self.run_watch([0.8])
        self.assertEqual(w.status()["last_event"]["kind"], "timeout")

    def test_no_reading_never_stops(self):
        w = ScriptedDockWatch([0.1])
        w.fetch = lambda: None
        with mock.patch.object(ebo_server, "DOCKWATCH_TIMEOUT_S", 0.2), mock.patch.object(ebo_server, "dockwatch", w):
            w.start()
            time.sleep(0.4)
        ev = w.status()["last_event"]
        self.assertEqual(ev["kind"], "timeout")
        self.assertGreater(ev["blind"], 0)
        self.assertEqual(self.stops(), 0)

    def test_falls_back_to_narrow_corridor(self):
        w = ScriptedDockWatch([0.5, 0.4, 0.3, 0.24, 0.2])
        orig = w.fetch

        def narrow():
            rd = orig()
            for k in ("free_ahead_wide_m", "free_ahead_wide_kind", "wide_half_width_m"):
                rd.pop(k)
            return rd
        w.fetch = narrow
        with mock.patch.object(ebo_server, "dockwatch", w):
            w.start()
            time.sleep(0.5)
        ev = w.status()["last_event"]
        self.assertEqual(ev["kind"], "stopped")
        self.assertIn("narrow", ev["corridor"])

    def dock_after(self, n):
        def on_poll(i):
            if i >= n:
                self.fake.charge = 1
        return on_poll

    def test_dock_action_starts_watch_and_notice_reaches_next_result(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        w = ScriptedDockWatch([0.6, 0.5, 0.4, 0.33, 0.27, 0.22])
        with mock.patch.object(ebo_server, "dockwatch", w):
            c = TestClient(ebo_server.app)
            r = c.post("/action/dock").json()
            self.assertTrue(r["ok"])
            self.assertTrue(r["dockwatch"]["watching"])
            self.assertEqual(self.fake.count(ebo_server.command_frame(ebo_server.CMD_DOCK)), 5)
            t0 = time.monotonic()
            while w.status()["active"] and time.monotonic() - t0 < 3:
                time.sleep(0.01)
            st = c.get("/dockwatch").json()
            self.assertEqual(st["last_event"]["kind"], "stopped")
            info = ebo_server.dockwatch_brief()
            self.assertEqual(info["last_event"]["kind"], "stopped")
            r2 = c.post("/move", json={"ly": 0, "rx": 0.3}).json()
            self.assertEqual(r2.get("dockwatch_notice"), "docking stopped: obstacle 0.22 m ahead")
            r3 = c.post("/move", json={"ly": 0, "rx": 0.3}).json()
            self.assertNotIn("dockwatch_notice", r3)

    def test_dock_action_without_watch(self):
        with mock.patch.object(ebo_server, "dockwatch", None), \
                mock.patch.object(ebo_server.time, "sleep", lambda _s: None):
            self.assertTrue(ebo_server.do_action("dock"))
        self.assertEqual(self.fake.count(ebo_server.command_frame(ebo_server.CMD_DOCK)), 5)



class TimedVideoBridge(FakeVideoBridge):
    """FakeVideoBridge that also records when each frame was sent (monotonic)."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.sent = []

    def send_rdt(self, frame):
        super().send_rdt(frame)
        with self.lock:
            self.sent.append((time.monotonic(), frame))

    def first(self, frame):
        with self.lock:
            return next((t for t, f in self.sent if f == frame), None)


class DockPrewakeTests(unittest.TestCase):
    """Before CMD_DOCK the guard wakes the stream and waits for a fresh floor-scaled reading
    (the dock watch was blind for a whole 6.8 s dock routine); it never refuses to dock."""

    def setUp(self):
        self.fake = TimedVideoBridge(charge=0, video=False)
        for p in (mock.patch.object(ebo_server, "bridge", self.fake),
                  mock.patch.object(ebo_server, "dockwatch", None),
                  mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", ""),
                  mock.patch.object(ebo_server, "WEB_USER", None)):
            p.start()
            self.addCleanup(p.stop)

    def scripted(self, script):
        g = ScriptedGuard(script)
        p = mock.patch.object(ebo_server, "guard", g)
        p.start()
        self.addCleanup(p.stop)
        return g

    def test_waits_for_scaled_reading_before_cmd_dock(self):
        marks = {}

        def script(t):
            if not self.fake.video:   # paused: no fresh reading until the wake starts the video
                return {"reason": "depth stale (9 s old)", "no_fresh_reading": True, "depth_running": True}
            if t < 0.4:               # fresh but the camera is still settling: no floor scale
                return {"reason": "distance ahead unknown (no floor scale)", "no_scale": True,
                        "free_ahead_m": None, "depth_running": True}
            marks.setdefault("scaled", time.monotonic())
            return {"free_ahead_m": 1.2, "depth_running": True}
        self.scripted(script)
        with mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 3.0):
            self.assertTrue(ebo_server.do_action("dock"))
        pre = ebo_server._last_dock_prewake
        self.assertTrue(pre["woke"], pre)
        self.assertTrue(pre["scaled"], pre)
        self.assertEqual(pre["free_ahead_m"], 1.2)
        self.assertEqual(self.fake.set_video_calls, 1)
        dock = ebo_server.command_frame(ebo_server.CMD_DOCK)
        self.assertEqual(self.fake.count(dock), 5)
        self.assertGreaterEqual(self.fake.first(dock), marks["scaled"])   # CMD_DOCK only after the reading

    def test_no_reading_within_limit_still_docks(self):
        self.scripted(lambda t: {"reason": "depth not running", "no_fresh_reading": True})
        with mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 0.3):
            t0 = time.monotonic()
            self.assertTrue(ebo_server.do_action("dock"))
            self.assertLess(time.monotonic() - t0, 2.0)
        pre = ebo_server._last_dock_prewake
        self.assertFalse(pre["reading"], pre)
        self.assertFalse(pre["scaled"], pre)
        self.assertEqual(self.fake.count(ebo_server.command_frame(ebo_server.CMD_DOCK)), 5)

    def test_reading_already_flowing_docks_at_once(self):
        self.fake.video = True
        self.scripted(lambda t: {"free_ahead_m": 0.9, "depth_running": True})
        with mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 3.0):
            self.assertTrue(ebo_server.do_action("dock"))
        pre = ebo_server._last_dock_prewake
        self.assertFalse(pre["woke"], pre)
        self.assertTrue(pre["scaled"], pre)
        self.assertLess(pre["waited_s"], 0.5)
        self.assertEqual(self.fake.set_video_calls, 0)

    def test_prewake_off(self):
        g = self.scripted(lambda t: {"reason": "depth not running", "no_fresh_reading": True})
        with mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 0.0), \
                mock.patch.object(ebo_server.time, "sleep", lambda _s: None):
            self.assertTrue(ebo_server.do_action("dock"))
        self.assertIsNone(ebo_server._last_dock_prewake)
        self.assertEqual(self.fake.set_video_calls, 0)

    def test_dock_response_and_watch_carry_prewake(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest("FastAPI test client unavailable")
        self.fake.video = True
        self.scripted(lambda t: {"free_ahead_m": 0.8, "depth_running": True})
        w = ScriptedDockWatch([0.8])
        with mock.patch.object(ebo_server, "dockwatch", w), \
                mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 2.0), \
                mock.patch.object(ebo_server, "DOCKWATCH_TIMEOUT_S", 0.3), \
                mock.patch.object(ebo_server, "DOCKWATCH_POLL_S", 0.01):
            r = TestClient(ebo_server.app).post("/action/dock").json()
            self.assertTrue(r["ok"])
            self.assertTrue(r["prewake"]["scaled"], r)
            self.assertEqual(w.status()["watch"]["prewake"]["free_ahead_m"], 0.8)
            t0 = time.monotonic()
            while w.status()["active"] and time.monotonic() - t0 < 2:
                time.sleep(0.01)
            ev = w.status()["last_event"]
            self.assertEqual(ev["prewake"]["free_ahead_m"], 0.8)
            self.assertIn("unscaled", ev)


if __name__ == "__main__":
    unittest.main()


LEFT_BLOCKED = [[-42, 1.5, "far"], [-34, 1.2, "clear"], [-25, 0.3, "obstacle"], [-17, 0.25, "obstacle"],
                [-8, 0.24, "obstacle"], [0, 0.24, "obstacle"], [8, 0.9, "clear"], [17, 1.2, "clear"],
                [25, 1.4, "clear"], [34, 2.0, "far"], [42, 2.0, "far"]]
RIGHT_BLOCKED = [[-d, m, k] for d, m, k in reversed(LEFT_BLOCKED)]
SYMMETRIC = [[-42, 1.0, "clear"], [-25, 0.3, "obstacle"], [-8, 0.24, "obstacle"], [0, 0.24, "obstacle"],
             [8, 0.24, "obstacle"], [25, 0.3, "obstacle"], [42, 1.0, "clear"]]
FAR_DECIDES = [[-42, 2.5, "far"], [-34, 2.4, "far"], [-25, 0.3, "obstacle"], [-8, 0.25, "obstacle"],
               [0, 0.24, "obstacle"], [8, 0.25, "obstacle"], [25, 0.3, "obstacle"], [34, 0.8, "clear"],
               [42, 0.7, "clear"]]


class CyclingDockWatch(ebo_server.DockWatch):
    """Every poll a fresh reading; distances cycle through an approach (so each new watch stops
    again); all readings carry the given depth sectors."""

    def __init__(self, sectors, cycle=(0.6, 0.5, 0.4, 0.3, 0.24, 0.2, 0.18, 0.16, 0.14)):
        super().__init__("http://127.0.0.1:9/none", None, None)
        self.sectors, self.cycle, self.i = sectors, list(cycle), 0

    def fetch(self):
        v = self.cycle[self.i % len(self.cycle)]
        self.i += 1
        return {"seq": self.i, "frame_t": time.time(), "free_ahead_m": v, "free_ahead_kind": "obstacle",
                "free_ahead_wide_m": v, "free_ahead_wide_kind": "obstacle", "wide_half_width_m": 0.17,
                "sectors": self.sectors}


class DockRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeVideoBridge(charge=0, video=True)
        self.moves, self.docks = [], []
        self.forward_ok = True

        def fake_move(ly, rx, duration=0.4):
            self.moves.append((round(ly, 2), round(rx, 2), round(duration, 2)))
            if ly > 0 and not self.forward_ok:
                return {"ok": False, "stopped_by_guard": True, "phase": "start", "reason": "obstacle 0.20 m"}
            return {"ok": True}
        for p in (mock.patch.object(ebo_server, "bridge", self.fake),
                  mock.patch.object(ebo_server, "guard", None),
                  mock.patch.object(ebo_server, "do_move", fake_move),
                  mock.patch.object(ebo_server, "DOCKWATCH_POLL_S", 0.005),
                  mock.patch.object(ebo_server, "DOCKWATCH_CONFIRM_S", 0.1),
                  mock.patch.object(ebo_server, "DOCKWATCH_TIMEOUT_S", 3.0),
                  mock.patch.object(ebo_server, "DOCKWATCH_RECOVER", True),
                  mock.patch.object(ebo_server, "DOCKWATCH_RECOVER_MAX", 2),
                  mock.patch.object(ebo_server, "DOCK_PREWAKE_S", 0),
                  mock.patch.object(ebo_server, "GUARD_VIDEO_KEEP_URL", ""),
                  mock.patch.object(ebo_server, "WEB_USER", None)):
            p.start()
            self.addCleanup(p.stop)

    def run_until(self, w, kinds, relaunch=False, timeout=6.0):
        ctx = [mock.patch.object(ebo_server, "dockwatch", w)]
        if not relaunch:
            ctx.append(mock.patch.object(ebo_server, "do_action", lambda name: self.docks.append(name) or True))
        for c in ctx:
            c.start()
        try:
            w.start()
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                ev = w.status()["last_event"]
                if ev and ev.get("kind") in kinds and not w.status()["active"]:
                    return ev
                time.sleep(0.01)
            return w.status()["last_event"]
        finally:
            for c in reversed(ctx):
                c.stop()

    def test_side_decision(self):
        self.assertEqual(ebo_server.dock_free_side(LEFT_BLOCKED)[0], "right")
        self.assertEqual(ebo_server.dock_free_side(RIGHT_BLOCKED)[0], "left")
        self.assertIsNone(ebo_server.dock_free_side(SYMMETRIC)[0])
        side, info = ebo_server.dock_free_side(FAR_DECIDES)
        self.assertEqual((side, info["basis"]), ("left", "far"))
        self.assertIsNone(ebo_server.dock_free_side(None)[0])

    def test_left_blocked_sidesteps_right_and_relaunches(self):
        w = CyclingDockWatch(LEFT_BLOCKED)
        ev = self.run_until(w, ("recovering",))
        self.assertEqual(ev["kind"], "recovering", ev)
        self.assertEqual(ev["side"], "right")
        r = ebo_server.DOCKWATCH_TURN_RX
        self.assertEqual(self.moves[0][:2], (0.0, round(r, 2)))           # turn right
        self.assertGreater(self.moves[1][0], 0)                          # forward sidestep
        self.assertEqual(self.moves[1][2], round(0.20 / 0.14, 2))
        self.assertEqual(self.moves[2][:2], (0.0, round(-r, 2)))         # turn back
        self.assertEqual(self.docks, ["dock"])
        self.assertEqual(w.next_attempt, 1)
        self.assertEqual(w.take_notice(),
                         "docking blocked: obstacle 0.24 m on the left - sidestepped right, retrying (1/2)")

    def test_right_blocked_sidesteps_left(self):
        w = CyclingDockWatch(RIGHT_BLOCKED)
        ev = self.run_until(w, ("recovering",))
        self.assertEqual(ev["side"], "left")
        self.assertLess(self.moves[0][1], 0)
        self.assertGreater(self.moves[2][1], 0)

    def test_forward_refused_aborts(self):
        self.forward_ok = False
        w = CyclingDockWatch(LEFT_BLOCKED)
        ev = self.run_until(w, ("recovery_aborted",))
        self.assertEqual(ev["kind"], "recovery_aborted", ev)
        self.assertEqual(len(self.moves), 2)          # turn + refused forward, no turn back
        self.assertEqual(self.docks, [])
        self.assertIn("sidestep forward was refused", w.take_notice())

    def test_ambiguous_does_not_move(self):
        w = CyclingDockWatch(SYMMETRIC)
        ev = self.run_until(w, ("stopped",))
        self.assertEqual(ev.get("recovery"), "ambiguous", ev)
        self.assertEqual(self.moves, [])
        self.assertEqual(self.docks, [])
        self.assertIn("no clearly freer side", w.take_notice())

    def test_max_attempts_then_failed(self):
        w = CyclingDockWatch(LEFT_BLOCKED)
        ev = self.run_until(w, ("failed",), relaunch=True, timeout=8.0)
        self.assertEqual(ev["kind"], "failed", ev)
        self.assertEqual(ev["attempt"], 2)
        self.assertEqual(len(self.moves), 6)          # 2 recoveries x (turn, forward, turn back)
        self.assertEqual(self.fake.count(ebo_server.command_frame(ebo_server.CMD_DOCK)), 10)  # 2 relaunches x 5
        n = w.take_notice()
        self.assertIn("retrying (1/2)", n)
        self.assertIn("retrying (2/2)", n)
        self.assertTrue(n.endswith("docking failed after 2 attempts: obstacle 0.24 m ahead"), n)

    def test_recovery_off_is_stop_only(self):
        with mock.patch.object(ebo_server, "DOCKWATCH_RECOVER", False):
            w = CyclingDockWatch(LEFT_BLOCKED)
            ev = self.run_until(w, ("stopped",))
        self.assertEqual(ev["kind"], "stopped")
        self.assertEqual(self.moves, [])
        self.assertEqual(self.docks, [])
        self.assertEqual(w.take_notice(), "docking stopped: obstacle 0.24 m ahead")

    def test_user_dock_resets_attempt(self):
        w = CyclingDockWatch(LEFT_BLOCKED)
        w.next_attempt = 0
        w.start()
        self.assertEqual(w.status()["watch"]["attempt"], 0)
        w.watch_id += 1   # supersede the thread
