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
