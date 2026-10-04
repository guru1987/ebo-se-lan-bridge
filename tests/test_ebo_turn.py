"""Closed-loop turn controller (app/ebo_turn.py) against a simulated EBO turn model.

The model is what the overhead 446 fps calibration measured (tools/turncal, runs 6-8, bare floor):
a pulse ends as a "blip" (7-27 deg) or a full turn (right ~50 +-10, left ~61 +-4); 0.12-0.18 s pulses
are a coin flip; short pulses are sometimes lost. The simulated camera measures each pulse with ~1 deg
noise and fails (like a real registration) beyond ~70 deg.
"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))
import ebo_turn  # noqa: E402


class SimRobot:
    def __init__(self, seed, lost_p=0.15, fail_registration_over=70.0, wrong_way=False):
        self.rng = random.Random(seed)
        self.heading = 0.0
        self.lost_p = lost_p
        self.fail_over = fail_registration_over
        self.wrong_way = wrong_way
        self.t = 0.0
        self.frames = {}
        self.next = 1
        self.last_pulse = 0.0
        self.refuse = False

    # --- bridge side
    def pulse(self, rx, secs):
        if self.refuse:
            return False, {"reason": "on the dock: only straight forward allowed"}, self.t
        side = 1 if rx > 0 else -1
        r = self.rng.random()
        if secs <= 0.09 and r < self.lost_p * 2:
            deg = 0.0
        elif secs <= 0.11 and r < self.lost_p:
            deg = 0.0
        elif secs < 0.12 or (secs < 0.18 and self.rng.random() < 0.5):
            deg = self.rng.uniform(7, 27)
        else:
            deg = self.rng.gauss(50, 10) if side > 0 else self.rng.gauss(61, 4)
        if self.wrong_way:
            side = -side
        self.last_pulse = side * deg
        self.heading += side * deg
        self.t += 1.0
        return True, {"ok": True}, self.t

    # --- vision side
    def still(self, after, max_wait):
        i = self.next
        self.next += 1
        self.frames[i] = self.heading
        return {"ok": True, "id": i, "t": self.t}

    def yaw(self, a, b):
        d = self.frames[b] - self.frames[a]
        if abs(d) > self.fail_over:
            return {"ok": False, "deg": 0.0}
        return {"ok": True, "deg": d + self.rng.gauss(0, 1.0), "ncc": 0.6}

    def chain(self, a, b, t0, t1):
        d = self.frames[b] - self.frames[a]
        return {"ok": True, "deg": d * self.rng.uniform(0.85, 1.0)}   # chaining reads a bit short


def run(target, seed, **kw):
    sim = SimRobot(seed, **{k: v for k, v in kw.items() if k in ("lost_p", "fail_registration_over", "wrong_way")})
    res = ebo_turn.run_turn(target, kw.get("tol", 5.0), kw.get("max_pulses", 10), sim.pulse, sim,
                            ebo_turn.load_config("/nonexistent"), log=lambda m: None)
    return res, sim


class TurnControllerTests(unittest.TestCase):
    def test_converges_for_many_targets(self):
        targets = [10, -10, 22.5, -22.5, 45, -45, 60, -60, 90, -90, 135, -135, 180, -180]
        ok = n = worst = 0
        for tg in targets:
            for seed in range(25):
                res, sim = run(tg, seed)
                n += 1
                true_err = sim.heading - tg
                if res["ok"]:
                    ok += 1
                    if not res["uncertain"]:
                        # the reported error must match the real one within measurement noise
                        self.assertLess(abs(true_err), res["tol_deg"] + 4.0, (tg, seed, res))
                worst = max(worst, abs(true_err))
                self.assertLessEqual(len(res["pulses"]), 10)
        rate = ok / n
        print(f"\n  closed-loop turns reported ok: {ok}/{n} ({rate:.0%}), worst true error {worst:.1f} deg")
        self.assertGreater(rate, 0.85)

    def test_small_target_already_done(self):
        res, sim = run(3.0, 1)
        self.assertTrue(res["ok"])
        self.assertEqual(res["pulses"], [])
        self.assertEqual(sim.heading, 0.0)

    def test_max_pulses_respected(self):
        res, _ = run(180, 2, max_pulses=2)
        self.assertLessEqual(len(res["pulses"]), 2)
        if not res["ok"]:
            self.assertIn("max_pulses", res["reason"])

    def test_refused_pulse_aborts(self):
        sim = SimRobot(3)
        sim.refuse = True
        res = ebo_turn.run_turn(45, 5, 10, sim.pulse, sim, ebo_turn.load_config("/nonexistent"), log=lambda m: None)
        self.assertFalse(res["ok"])
        self.assertIn("refused", res["reason"])
        self.assertEqual(sim.heading, 0.0)

    def test_wrong_way_aborts(self):
        res, _ = run(45, 4, wrong_way=True)
        self.assertFalse(res["ok"])
        self.assertIn("wrong way", res["reason"])
        self.assertEqual(len(res["pulses"]), 1)

    def test_no_camera_aborts_before_moving(self):
        sim = SimRobot(5)
        sim.still = lambda after, w: {"ok": False, "reason": "no fresh frames"}
        res = ebo_turn.run_turn(45, 5, 10, sim.pulse, sim, ebo_turn.load_config("/nonexistent"), log=lambda m: None)
        self.assertFalse(res["ok"])
        self.assertEqual(sim.heading, 0.0)

    def test_failed_registration_uses_chain_then_nominal(self):
        res, _ = run(150, 6, fail_registration_over=10.0)   # every full turn "too large"
        methods = {p["method"] for p in res["pulses"]}
        self.assertIn("chain", methods)
        sim = SimRobot(7, fail_registration_over=10.0)
        sim.chain = lambda a, b, t0, t1: {"ok": False}
        res2 = ebo_turn.run_turn(150, 5, 10, sim.pulse, sim, ebo_turn.load_config("/nonexistent"), log=lambda m: None)
        self.assertTrue(res2["uncertain"])
        self.assertIn("nominal", {p["method"] for p in res2["pulses"]})

    def test_config_override_file(self):
        import json
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"dur": {"blip": {"left": 0.12}}, "settle_s": 0.6}, f)
        cfg = ebo_turn.load_config(f.name)
        os.unlink(f.name)
        self.assertEqual(cfg["dur"]["blip"]["left"], 0.12)
        self.assertEqual(cfg["dur"]["blip"]["right"], 0.10)
        self.assertEqual(cfg["settle_s"], 0.6)


if __name__ == "__main__":
    unittest.main()


# ---------------------------------------------------------------- delayed stream (live 2026-10-04)
import types  # noqa: E402


class SimDelayed:
    """Continuous-time EBO + camera stream with latency (0.3-1.5 s, per-pulse jitter), 30 fps.
    The camera shows the heading of (t - latency). Reproduces the live failure: a fixed wait after
    the pulse grabs frames from BEFORE the motion appears in the stream."""

    FPS = 30.0

    def __init__(self, seed, lat_lo=0.3, lat_hi=1.5, frozen=False):
        self.rng = random.Random(seed)
        self.now = 0.0
        self.segs = []          # (t0, t1, h0, h1) true heading ramps
        self.heading = 0.0
        self.lat_lo, self.lat_hi = lat_lo, lat_hi
        self.lat = self.rng.uniform(lat_lo, lat_hi)
        self.frozen = frozen
        self.seed = seed
        self._cam, self._vmax = {}, -1e9
        self.frames = {}
        self.next = 1
        self.base = SimRobot(seed + 1000)     # reuse the two-mode pulse model

    # clock used by ebo_turn (patched in)
    def monotonic(self):
        return self.now

    def true_heading(self, t):
        h = 0.0
        for t0, t1, h0, h1 in self.segs:
            if t >= t1:
                h = h1
            elif t > t0:
                return h0 + (h1 - h0) * (t - t0) / (t1 - t0)
            else:
                return h0
        return h

    def cam(self, t):
        """Heading shown by the frame that ARRIVES at time t. Deterministic per frame, and the shown
        moment never goes backwards (a stream delays frames, it does not reorder them)."""
        k = round(t * self.FPS)
        if k not in self._cam:
            jit = random.Random(k * 7919 + self.seed).uniform(-0.03, 0.03)
            v = max(t - self.lat - jit, self._vmax)
            self._vmax = v
            self._cam[k] = self.true_heading(v)
        return self._cam[k]

    def frame_times(self, after, upto):
        if self.frozen:
            return []
        k0 = int(after * self.FPS) + 1
        return [k / self.FPS for k in range(k0, int(upto * self.FPS) + 1)]

    def pulse(self, rx, secs):
        self.lat = self.rng.uniform(self.lat_lo, self.lat_hi)     # jitter per pulse (wake etc.)
        ok, info, _ = self.base.pulse(rx, secs)
        deg = self.base.last_pulse
        t0 = self.now + 0.6 + 0.05                                 # warm-up + onset
        dur = max(0.05, abs(deg) / 420.0)
        self.segs.append((t0, t0 + dur, self.heading, self.heading + deg))
        self.heading += deg
        self.now += 0.6 + secs + 0.01
        return ok, info, self.now

    def _keep(self, t):
        i = self.next
        self.next += 1
        self.frames[i] = self.cam(t)
        return i

    def still(self, after, max_wait):
        end = self.now + max_wait
        while self.now < end:
            ts = self.frame_times(after, self.now)
            hs = [self.cam(t) for t in ts]
            for j in range(1, len(ts)):
                if abs(hs[j] - hs[j - 1]) < 0.5:
                    return {"ok": True, "id": self._keep(ts[j]), "t": ts[j]}
            self.now += 0.05
        return {"ok": False, "reason": "no fresh frames"}

    def settle(self, before_id, t_start, t_end, cfg):
        hb = self.frames[before_id]
        while True:
            ts = self.frame_times(t_start, self.now)
            hs = [self.cam(t) for t in ts]
            meta = [(t, abs(h - hb) > 1.5, j > 0 and abs(h - hs[j - 1]) < 0.5) for j, (t, h) in enumerate(zip(ts, hs))]
            state, idx, info = ebo_turn.settle_pick(meta, self.now, t_start, t_end, cfg)
            if state == "ok":
                tc = info["t_change"]
                return {"ok": True, "id": self._keep(meta[idx][0]), "t": meta[idx][0], "moved": info["moved"],
                        "latency_s": None if tc is None else tc - t_start, "settle_s": meta[idx][0] - t_end}
            if state == "timeout":
                return {"ok": False, "reason": "picture did not settle" if meta else "no fresh frames"}
            self.now += 0.05

    def yaw(self, a, b):
        d = self.frames[b] - self.frames[a]
        if abs(d) > 70.0:
            return {"ok": False, "deg": 0.0}
        return {"ok": True, "deg": d + self.rng.gauss(0, 1.0), "ncc": 0.6}

    def chain(self, a, b, t0, t1):
        d = self.frames[b] - self.frames[a]
        return {"ok": True, "deg": d * self.rng.uniform(0.85, 1.0)}


def run_delayed(target, seed, mode="settle", **kw):
    sim = SimDelayed(seed, **kw)
    cfg = ebo_turn.load_config("/nonexistent")
    cfg["settle_mode"] = mode
    real_time = ebo_turn.time
    ebo_turn.time = types.SimpleNamespace(monotonic=sim.monotonic, sleep=lambda s: None)
    try:
        res = ebo_turn.run_turn(target, 5.0, 10, sim.pulse, sim, cfg, log=lambda m: None)
    finally:
        ebo_turn.time = real_time
    return res, sim


class DelayedStreamTests(unittest.TestCase):
    TARGETS = [22.5, -22.5, 45, -45, 90, -90, 135, -135, 180, -180]

    def test_old_fixed_wait_lags_one_pulse(self):
        """Reproduce the live log: with a fixed 0.45 s wait the first pulse reads ~0 and the error is
        carried into the next pulses; true error is large."""
        zero_first = bad = n = 0
        for tg in (90, -90):
            for seed in range(20):
                res, sim = run_delayed(tg, seed, mode="still", lat_lo=0.8, lat_hi=1.2)
                n += 1
                ps = res["pulses"]
                if ps and ps[0]["measured_deg"] is not None and abs(ps[0]["measured_deg"]) < 3:
                    zero_first += 1
                if abs(sim.heading - tg) > 5:
                    bad += 1
        print(f"\n  old fixed wait @0.8-1.2 s latency: first pulse read ~0 in {zero_first}/{n}, "
              f"true error >5 deg in {bad}/{n}")
        self.assertGreater(zero_first, n * 0.8)
        self.assertGreater(bad, n * 0.5)

    def test_settle_converges_with_latency(self):
        ok = n = 0
        worst = 0.0
        lats = []
        for tg in self.TARGETS:
            for seed in range(20):
                res, sim = run_delayed(tg, seed)
                n += 1
                err = abs(sim.heading - tg)
                worst = max(worst, err)
                if res["ok"] and err <= 5.0 + 1.5:     # measurement noise ~1 deg
                    ok += 1
                lats += [p["latency_s"] for p in res["pulses"] if p.get("latency_s") is not None]
        rate = ok / n
        print(f"\n  settle @0.3-1.5 s latency: ok within 5 deg {ok}/{n} ({rate:.0%}), worst true error "
              f"{worst:.1f}; measured latency {min(lats):.2f}-{max(lats):.2f} s")
        self.assertGreaterEqual(rate, 0.90)

    def test_settle_reports_latency_and_after_end(self):
        res, sim = run_delayed(45, 3, lat_lo=1.0, lat_hi=1.0)
        p = res["pulses"][0]
        self.assertTrue(p["moved"])
        self.assertGreater(p["latency_s"], 0.9)
        self.assertGreater(p["settle_s"], 0.5)

    def test_lost_pulse_reads_zero_after_no_motion_window(self):
        sim = SimDelayed(4, lat_lo=0.9, lat_hi=0.9)
        sim.base.pulse = lambda rx, secs: (True, {"ok": True}, 0.0)   # nothing moves
        sim.base.last_pulse = 0.0
        cfg = ebo_turn.load_config("/nonexistent")
        real_time = ebo_turn.time
        ebo_turn.time = types.SimpleNamespace(monotonic=sim.monotonic, sleep=lambda s: None)
        try:
            res = ebo_turn.run_turn(20, 5.0, 3, sim.pulse, sim, cfg, log=lambda m: None)
        finally:
            ebo_turn.time = real_time
        self.assertTrue(all(p["moved"] is False and abs(p["measured_deg"]) < 3 for p in res["pulses"]))
        self.assertTrue(all(p["settle_s"] >= cfg["no_motion_s"] - 0.05 for p in res["pulses"]))

    def test_frozen_stream_times_out_without_counting(self):
        sim = SimDelayed(5)
        cfg = ebo_turn.load_config("/nonexistent")
        real_time = ebo_turn.time
        ebo_turn.time = types.SimpleNamespace(monotonic=sim.monotonic, sleep=lambda s: None)
        try:
            ref = sim.still(-1.0, 1.0)            # one frame exists before freezing
            sim.frozen = True
            sim.still = lambda after, w, _r=ref: _r   # start frame available, then the stream stops
            res = ebo_turn.run_turn(45, 5.0, 5, sim.pulse, sim, cfg, log=lambda m: None)
        finally:
            ebo_turn.time = real_time
        self.assertFalse(res["ok"])
        self.assertEqual(len(res["pulses"]), 1)
        self.assertIsNone(res["pulses"][0]["measured_deg"])
        self.assertIn("camera", res["reason"])


class BlipAdaptTests(unittest.TestCase):
    def test_blip_that_became_full_turn_gets_shorter_and_short_left_blips_are_used(self):
        """Juraj 2026-10-04: left 0.06-0.13 s give small steps. A right blip that comes out as a full
        turn must shorten the right blip; short left blips must be tried, not excluded."""
        sim = SimRobot(9)
        script = {}

        def pulse(rx, secs):
            if rx > 0:
                deg = 55.0 if secs >= 0.10 else 12.0
            else:
                deg = -9.0 if secs >= 0.06 else 0.0
            sim.heading += deg
            sim.t += 1.0
            script.setdefault("calls", []).append((rx, secs, deg))
            return True, {"ok": True}, sim.t
        sim.pulse = pulse
        res = ebo_turn.run_turn(30, 5, 10, pulse, sim, ebo_turn.load_config("/nonexistent"), log=lambda m: None)
        self.assertLess(res["blip_dur"]["right"], 0.10)
        left = [c for c in script["calls"] if c[0] < 0]
        self.assertTrue(left, script)
        self.assertTrue(res["ok"], res)


class SettlePickTests(unittest.TestCase):
    CFG = dict(settle_min_s=0.5, settle_max_s=3.0, no_motion_s=1.5, stable_n=3, stable_s=0.3)

    def test_waits_before_min(self):
        st, _, _ = ebo_turn.settle_pick([(1.1, True, True)] * 3, 1.2, 0.0, 1.0, self.CFG)
        self.assertEqual(st, "wait")

    def test_pre_motion_still_frames_are_not_accepted(self):
        # still frames right after END but before the change shows up must not be picked
        fr = [(1.0 + k / 30, False, True) for k in range(1, 20)]
        st, _, _ = ebo_turn.settle_pick(fr, 1.7, 0.0, 1.0, self.CFG)
        self.assertEqual(st, "wait")

    def test_picks_after_change_and_stability(self):
        fr = [(1.0 + k / 30, False, True) for k in range(1, 10)]          # pre-motion
        fr += [(1.3 + k / 30, True, False) for k in range(1, 8)]           # moving (changed, not still)
        fr += [(1.6 + k / 30, True, True) for k in range(1, 15)]           # settled
        st, i, info = ebo_turn.settle_pick(fr, 2.2, 0.0, 1.0, self.CFG)
        self.assertEqual(st, "ok")
        self.assertGreater(fr[i][0], 1.6)
        self.assertTrue(info["moved"])

    def test_timeout(self):
        fr = [(1.0 + k / 30, True, False) for k in range(1, 100)]          # never still
        st, _, _ = ebo_turn.settle_pick(fr, 4.1, 0.0, 1.0, self.CFG)
        self.assertEqual(st, "timeout")
