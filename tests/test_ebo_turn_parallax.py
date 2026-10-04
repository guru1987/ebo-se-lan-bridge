"""/turn parallax correction: the camera sits 48 mm in front of the turning centre, so near objects
shift more than the robot turns. Live 2026-10-04 closures read ~+4 % (right circle); this checks what
parallax at >= 0.5 m can and cannot explain, and that the correction removes it."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))
import ebo_turn  # noqa: E402

CFG = ebo_turn.load_config("/nonexistent")
DEGS = [-42, -33, -25, -17, -8, 0, 8, 17, 25, 33, 42]


def scene(near_side, near_m=0.5, far_m=3.0):
    """near objects on one side (sofa > 0.5 m), the room far on the other."""
    out = []
    for d in DEGS:
        near = (d > 0) if near_side == "right" else (d < 0)
        out.append([d, near_m if near else far_m, "obstacle"])
    return out


class ParallaxModel(unittest.TestCase):
    def test_parallax_only_over_reads_and_size_at_half_a_metre(self):
        rows = []
        for d_m in (0.5, 0.75, 1.0, 2.0, 3.0):
            sec = [[d, d_m, "obstacle"] for d in DEGS]
            for th in (15, 45, 90):
                for sgn in (1, -1):
                    a = ebo_turn.apparent_yaw(sgn * th, sec, CFG)
                    self.assertGreaterEqual(abs(a), th - 1e-6)        # never under-reads
                rows.append((d_m, th, ebo_turn.apparent_yaw(th, sec, CFG) / th))
        print("\n  parallax ratio (apparent/true), r = 48 mm, uniform range:")
        for d_m, th, q in rows:
            print(f"    d {d_m:4.2f} m  theta {th:2d}  {q:.3f}")
        q05 = ebo_turn.apparent_yaw(45, [[d, 0.5, "obstacle"] for d in DEGS], CFG) / 45
        self.assertTrue(1.07 < q05 < 1.11, q05)          # ~ +9 % at 0.5 m
        q1 = ebo_turn.apparent_yaw(45, [[d, 1.0, "obstacle"] for d in DEGS], CFG) / 45
        self.assertTrue(1.03 < q1 < 1.06, q1)            # ~ +4.5 % at 1 m (the live right circle: +4 %)

    def test_asymmetric_scene_both_directions_over_read(self):
        for side in ("right", "left"):
            sec = scene(side)
            for th in (30, 60, 90):
                ar, al = ebo_turn.apparent_yaw(th, sec, CFG), ebo_turn.apparent_yaw(-th, sec, CFG)
                self.assertGreater(ar, th)
                self.assertLess(al, -th)      # left reads MORE left: a left under-read is not parallax

    def test_correction_recovers_true_yaw(self):
        worst = 0.0
        for side in ("right", "left"):
            for near in (0.5, 0.7, 1.0):
                sec = scene(side, near_m=near)
                for th in list(range(10, 91, 10)):
                    for sgn in (1, -1):
                        true = sgn * th
                        meas = ebo_turn.apparent_yaw(true, sec, CFG)
                        corr = ebo_turn.parallax_true_yaw(meas, sec, CFG)
                        worst = max(worst, abs(corr - true))
        print(f"\n  parallax correction worst error over 0.5-1.0 m scenes, 10-90 deg both ways: {worst:.3f} deg")
        self.assertLess(worst, 0.5)

    def test_no_depth_means_no_change(self):
        self.assertEqual(ebo_turn.parallax_true_yaw(42.0, None, CFG), 42.0)
        cfg = dict(CFG, parallax=False)
        self.assertEqual(ebo_turn.parallax_true_yaw(42.0, scene("right"), cfg), 42.0)


class ParallaxClosedLoop(unittest.TestCase):
    """run_turn against a robot whose camera reads parallax-enlarged yaws (near objects 0.5 m on one side)."""

    class Sim:
        def __init__(self, sec):
            self.sec, self.h, self.frames, self.next, self.t = sec, 0.0, {}, 1, 0.0
            self.sizes = {"full": 55.0, "blip": 14.0}

        def pulse(self, rx, secs):
            deg = self.sizes["full"] if secs >= 0.18 else self.sizes["blip"]
            self.h += deg if rx > 0 else -deg
            self.t += 1.0
            return True, {"ok": True}, self.t

        def still(self, after, max_wait, hold_s=0.0, hold_n=2):
            i = self.next
            self.next += 1
            self.frames[i] = self.h
            return {"ok": True, "id": i, "t": self.t}

        def yaw(self, a, b):
            true = self.frames[b] - self.frames[a]
            if abs(true) > 70:
                return {"ok": False, "deg": 0.0}
            return {"ok": True, "deg": ebo_turn.apparent_yaw(true, self.sec, CFG), "ncc": 0.9}

        def chain(self, a, b, t0, t1):
            return {"ok": False}

    def _run(self, target, side, parallax):
        sim = self.Sim(scene(side))
        cfg = dict(CFG, parallax=parallax, settle_mode="still")
        res = ebo_turn.run_turn(target, 5.0, 12, sim.pulse, sim, cfg, log=lambda m: None,
                                depth=lambda: sim.sec)
        return res, sim.h

    def test_closed_loop_true_turn_with_and_without_correction(self):
        print("\n  closed loop vs a 0.5 m-near scene (true turned, deg):")
        for side in ("right", "left"):
            for target in (90, -90, 45, -45):
                r0, h0 = self._run(target, side, False)
                r1, h1 = self._run(target, side, True)
                print(f"    near {side:5s} target {target:+4d}: uncorrected true {h0:+7.1f} (believed "
                      f"{r0['turned_deg']:+6.1f})  corrected true {h1:+7.1f} (believed {r1['turned_deg']:+6.1f})")
                # the point of the correction: what /turn believes it turned = what it really turned
                # (the fixed 14/55 deg pulse sizes of this fake limit how close it can get to the target)
                self.assertLessEqual(abs(r1["turned_deg"] - h1), 1.0)


if __name__ == "__main__":
    unittest.main()
