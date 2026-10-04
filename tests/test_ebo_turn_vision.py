"""Offline accuracy test of tools/ebo_turn_vision.py with synthetic turns rendered from real robot frames.

Needs numpy + OpenCV (skipped otherwise; the bridge venv has neither - run with a Python that does,
e.g. EBO_TURN_TEST_FRAMES=/path/to/frames python -m pytest tests/test_ebo_turn_vision.py).
A "turned" view is rendered from a real fisheye frame by rotating every camera ray about the levelled
vertical axis (camera pitched up 15 deg), i.e. exactly the motion /turn has to measure (minus the
small parallax of the real robot, whose camera sits a few cm off the rotation axis).
"""
import glob
import math
import os
import sys
import unittest

try:
    import numpy as np
    import cv2
except ImportError:  # pragma: no cover
    np = cv2 = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
LENS = os.path.join(ROOT, "config", "ebo_se_fisheye_1920.json")
PITCH = 15.0
FRAMES = os.environ.get("EBO_TURN_TEST_FRAMES", "/home/cuda/mythos_lab/porch/tmp/calib")


def render_turned(img, yaw_right_deg, K, D, theta_max, pitch=PITCH, rng=None):
    """Fisheye view after the robot turned right by yaw (deg) on the spot, from source fisheye img."""
    import ebo_turn_vision as tv
    h, w = img.shape[:2]
    sc = w / 1920.0
    Ks = K.copy(); Ks[:2, :] *= sc
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    pts = np.stack([uu.ravel(), vv.ravel()], 1).reshape(-1, 1, 2)
    und = cv2.fisheye.undistortPoints(pts, Ks, D).reshape(-1, 2)
    cam = np.concatenate([und, np.ones((len(und), 1))], 1)
    a = math.radians(pitch); ca, sa = math.cos(a), math.sin(a)
    lev = np.stack([cam[:, 0], ca * cam[:, 1] - sa * cam[:, 2], sa * cam[:, 1] + ca * cam[:, 2]], 1)
    t = math.radians(yaw_right_deg); ct, st = math.cos(t), math.sin(t)
    old = np.stack([ct * lev[:, 0] + st * lev[:, 2], lev[:, 1], -st * lev[:, 0] + ct * lev[:, 2]], 1)
    camo = tv.level_to_camera(old, pitch)
    uv = tv.fisheye_project(camo, Ks, D, theta_max)
    mx = np.nan_to_num(uv[:, 0], nan=-1).reshape(h, w).astype(np.float32)
    my = np.nan_to_num(uv[:, 1], nan=-1).reshape(h, w).astype(np.float32)
    out = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    miss = (mx < 0) | (my < 0)
    rng = rng or np.random.default_rng(0)
    out[miss] = rng.integers(60, 120, int(miss.sum()))   # new, unseen scene: unrelated texture
    return out


def real_frames():
    out = []
    for f in sorted(glob.glob(os.path.join(FRAMES, "*.jpg"))):
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if im is None or im.shape[1] not in (1280, 1920) or im.shape[1] * 9 != im.shape[0] * 16:
            continue
        if "overlay" in f or "marked" in f or "crop" in f or "_s." in f or f.endswith("s.jpg"):
            continue
        if im.std() < 25:
            continue
        out.append((os.path.basename(f), cv2.resize(im, (960, 540), interpolation=cv2.INTER_AREA)))
    return out


def noisy(img, k, amp=3):
    """A real camera never delivers byte-identical frames: add per-frame sensor noise."""
    rng = np.random.default_rng(k)
    return np.clip(img.astype(np.int16) + rng.integers(-amp, amp + 1, img.shape), 0, 255).astype(np.uint8)


@unittest.skipIf(cv2 is None, "needs numpy + OpenCV")
class TurnVisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import ebo_turn_vision as tv
        cls.tv = tv
        cls.K, cls.D, _, cls.tmax = tv.load_lens(LENS)
        cls.mk = tv.StripMaker(LENS, PITCH, 960, 540)
        cls.frames = real_frames()[:6]

    def test_have_frames(self):
        self.assertGreaterEqual(len(self.frames), 3, f"need real robot frames in {FRAMES}")

    def test_known_yaw_accuracy(self):
        errs, fails = [], []
        for name, img in self.frames:
            s0, v = self.mk(img)
            for yaw in (-55, -40, -25, -12, -5, 0, 5, 12, 25, 40, 55):
                s1, _ = self.mk(render_turned(img, yaw, self.K, self.D, self.tmax))
                r = self.tv.yaw_between(s0, s1, v)
                if not r["ok"]:
                    fails.append((name, yaw, r))
                    continue
                errs.append(abs(r["deg"] - yaw))
                self.assertLess(abs(r["deg"] - yaw), 3.0, (name, yaw, r))
        print(f"\n  yaw error over {len(errs)} cases: mean {np.mean(errs):.2f} max {np.max(errs):.2f} deg; "
              f"not ok: {len(fails)}")
        self.assertLessEqual(len(fails), max(1, len(errs) // 15), fails[:3])
        self.assertLess(float(np.mean(errs)), 1.0)

    def test_sign_right_is_positive(self):
        name, img = self.frames[0]
        s0, v = self.mk(img)
        s1, _ = self.mk(render_turned(img, 20, self.K, self.D, self.tmax))
        self.assertGreater(self.tv.yaw_between(s0, s1, v)["deg"], 15)

    def test_too_large_is_reported_not_ok(self):
        bad = 0
        for name, img in self.frames:
            s0, v = self.mk(img)
            for yaw in (95, -95, 120):
                s1, _ = self.mk(render_turned(img, yaw, self.K, self.D, self.tmax))
                r = self.tv.yaw_between(s0, s1, v)
                if r["ok"] and abs(r["deg"] - yaw) > 5:
                    bad += 1
        self.assertEqual(bad, 0, "a turn beyond the overlap must not be reported as a wrong ok angle")

    def test_service_still_and_yaw_with_fake_source(self):
        tv = self.tv
        name, img = self.frames[0]
        clock = {"t": 100.0}

        class Src:
            w, h, url, err = 960, 540, "fake", None
            def __init__(self, url): self.frames = []
            def start(self): pass
            def alive(self): return True
            def stop(self): pass
            def since(self, t): return [f for f in self.frames if f[0] > t]

        svc = tv.Service(source_factory=Src, clock=lambda: clock["t"], sleep=lambda s: clock.__setitem__("t", clock["t"] + s))
        svc.src = Src("fake")
        svc.src.frames = [(100.1, noisy(img, 1)), (100.13, noisy(img, 2))]
        svc.mk = self.mk
        a = svc.still(100.0, 1.0)
        self.assertTrue(a["ok"], a)
        t2 = render_turned(img, -30, self.K, self.D, self.tmax)
        svc.src.frames += [(101.0, noisy(t2, 3)), (101.03, noisy(t2, 4))]
        b = svc.still(100.5, 1.0)
        self.assertTrue(b["ok"], b)
        r = svc.yaw(a["id"], b["id"])
        self.assertTrue(r["ok"], r)
        self.assertAlmostEqual(r["deg"], -30, delta=2.0)
        # moving frames only (different every frame) -> no still frame
        svc.src.frames += [(102.0 + i * 0.03, render_turned(img, i * 8, self.K, self.D, self.tmax)) for i in range(4)]
        self.assertFalse(svc.still(101.9, 0.2)["ok"])

    def test_service_settle_skips_pre_motion_frames(self):
        """Live 2026-10-04: after a pulse the stream still shows the OLD view for ~1 s. settle must not
        take those still pre-motion frames; it waits for the change and the new still view."""
        tv = self.tv
        name, img = self.frames[0]
        clock = {"t": 200.0}

        class Src:
            w, h, url, err = 960, 540, "fake", None
            def __init__(self, url): self.frames = []
            def start(self): pass
            def alive(self): return True
            def stop(self): pass
            def since(self, t): return [f for f in self.frames if f[0] > t and f[0] <= clock["t"]]

        svc = tv.Service(source_factory=Src, clock=lambda: clock["t"],
                         sleep=lambda s: clock.__setitem__("t", clock["t"] + s))
        svc.src = Src("fake")
        svc.mk = self.mk
        turned = render_turned(img, 25, self.K, self.D, self.tmax)
        mid = render_turned(img, 12, self.K, self.D, self.tmax)
        fr = [(200.0 + k / 30, noisy(img, k)) for k in range(1, 6)]             # before
        t_start, t_end = 200.2, 200.5
        fr += [(200.2 + k / 30, noisy(img, 100 + k)) for k in range(1, 30)]     # stream still shows the old view ~1 s
        fr += [(201.2, mid), (201.233, turned)]                                 # motion appears
        fr += [(201.233 + k / 30, noisy(turned, 200 + k)) for k in range(1, 20)]  # new still view
        svc.src.frames = fr
        a = svc.still(199.9, 0.5)
        self.assertTrue(a["ok"], a)
        cfg = dict(settle_min_s=0.5, settle_max_s=3.0, no_motion_s=1.5, stable_n=3, stable_s=0.3)
        clock["t"] = t_end
        b = svc.settle(a["id"], t_start, t_end, cfg)
        self.assertTrue(b["ok"], b)
        self.assertTrue(b["moved"])
        self.assertGreater(b["t"], 201.23)
        self.assertAlmostEqual(b["latency_end_s"], 0.7, delta=0.05)
        r = svc.yaw(a["id"], b["id"])
        self.assertAlmostEqual(r["deg"], 25, delta=2.0)
        # the old fixed-wait still() would have taken a pre-motion frame here
        old = svc.still(t_end + 0.45, 0.1)
        self.assertAlmostEqual(svc.yaw(a["id"], old["id"])["deg"], 0, delta=1.0)


    def _svc(self, t0):
        tv = self.tv
        clock = {"t": t0}

        class Src:
            w, h, url, err = 960, 540, "fake", None
            def __init__(self, url): self.frames = []
            def start(self): pass
            def alive(self): return True
            def stop(self): pass
            def since(self, t): return [f for f in self.frames if f[0] > t and f[0] <= clock["t"]]

        svc = tv.Service(source_factory=Src, clock=lambda: clock["t"],
                         sleep=lambda s: clock.__setitem__("t", clock["t"] + s))
        svc.src = Src("fake")
        svc.mk = self.mk
        return svc, clock

    def test_frozen_stretch_is_not_rest_and_small_full_pulse_waits(self):
        """Live 2026-10-04: a full pulse 'settled' at +5.6 deg because a frozen stretch (repeated picture)
        followed a small leftover change. Repeats must not count as stillness, and a full pulse reading
        < 30 deg keeps waiting for the real motion."""
        name, img = self.frames[0]
        svc, clock = self._svc(300.0)
        small = render_turned(img, 5.6, self.K, self.D, self.tmax)
        big = render_turned(img, 52.0, self.K, self.D, self.tmax)
        fr = [(300.0 + k / 30, noisy(img, k)) for k in range(1, 20)]            # at rest before
        t_start, t_end = 300.7, 301.5
        fr += [(300.75, noisy(small, 50))]                                     # leftover small change
        fr += [(300.75 + k / 30, small) for k in range(1, 25)]                  # FROZEN: identical repeats
        fr += [(301.6 + k / 30, noisy(small, 300 + k)) for k in range(0, 12)]   # stream back, still small ...
        fr += [(302.05, render_turned(img, 30, self.K, self.D, self.tmax))]     # ... then the real motion
        fr += [(302.08 + k / 30, noisy(big, 400 + k)) for k in range(0, 40)]    # new still view
        svc.src.frames = fr
        a = svc.still(300.0, 0.5, 0.3, 5)
        self.assertTrue(a["ok"], a)
        cfg = dict(settle_min_s=0.8, settle_max_s=3.5, no_motion_s=2.5, stable_n=5, stable_s=0.5,
                   distinct_only=True, dup_gap_s=0.012, plaus_min_deg={"full": 30.0, "blip": 3.0},
                   plaus_hold_s=1.5, kind="full")
        clock["t"] = t_end
        b = svc.settle(a["id"], t_start, t_end, cfg)
        self.assertTrue(b["ok"], b)
        self.assertGreater(b["t"], 302.08)
        self.assertAlmostEqual(svc.yaw(a["id"], b["id"])["deg"], 52, delta=2.0)

    def test_still_hold_needs_a_rest_period(self):
        name, img = self.frames[0]
        svc, clock = self._svc(400.0)
        moving = [(400.0 + k / 30, noisy(render_turned(img, k * 2.0, self.K, self.D, self.tmax), k))
                  for k in range(1, 10)]
        rest = [(400.3 + k / 30, noisy(render_turned(img, 18.0, self.K, self.D, self.tmax), 100 + k))
                for k in range(1, 25)]
        svc.src.frames = moving + rest
        clock["t"] = 400.45                     # only ~0.15 s of rest so far
        self.assertFalse(svc.still(400.0, 0.0, 0.5, 5)["ok"])
        clock["t"] = 400.95
        r = svc.still(400.0, 0.5, 0.5, 5)
        self.assertTrue(r["ok"], r)
        self.assertGreaterEqual(r["still_s"], 0.5)


if __name__ == "__main__":
    unittest.main()
