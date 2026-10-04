#!/usr/bin/env python3
"""Yaw measurement for POST /turn: how far did the robot turn between two still frames of its own camera?

Runs as a helper process of the bridge (the bridge's venv has no numpy/OpenCV): started with --serve,
it reads JSON requests line by line on stdin and answers one JSON line each on stdout.

Method: every still frame is reprojected into a LEVELLED equirectangular strip (azimuth x elevation,
fixed degrees per pixel) using the calibrated fisheye model and the camera's measured pitch (15 deg
up). A turn of the robot about its vertical axis is then a pure horizontal shift of that strip, so
the yaw is the shift that best aligns the "before" and "after" strips (normalised cross-correlation of
gradient images over their overlap). Accurate to ~1 deg while the views overlap; beyond ~70 deg the
overlap is too small and the result is reported as not ok (peak too weak / overlap too narrow).
Sign: + = the robot turned RIGHT (clockwise seen from above) = the scene moved left in the image.

Requests (one JSON object per line):
  {"cmd": "start", "url": "rtsp://...", "lens": "...json", "pitch_deg": 15}  start the stream decoder
  {"cmd": "still", "after": <time.monotonic()>, "max_wait": 3.0}          next still frame newer than
                                                                            `after` -> {"id", "t"}
  {"cmd": "yaw", "a": id, "b": id}                                          -> {"deg", "ncc", ...}
  {"cmd": "chain", "a": id, "b": id, "t0": ..., "t1": ...}                  integrate through the
                                                                            frames between t0..t1
  {"cmd": "stop"}
time.monotonic() is CLOCK_MONOTONIC, shared by all processes of the host, so the bridge's timestamps
can be compared with the frame arrival times here.
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "app"))
from ebo_turn import settle_pick, plausible, distinct_frames  # noqa: E402  (pure decision logic,
#                                                    shared with the controller tests)

DECODE_W, DECODE_H = 960, 540          # decoder output (the stream is 1920x1080)
PPD = 4.0                               # strip resolution, pixels per degree
AZ = (-50.0, 50.0)                      # strip azimuth range (deg, levelled)
EL = (-6.0, 40.0)                       # strip elevation range (deg): mostly above the floor, which
                                        # has strong parallax this close to the camera
MAX_SHIFT = 80.0                        # searched yaw range (deg)
MIN_OVERLAP = 22.0                      # minimum overlap of valid columns (deg)
MIN_NCC = 0.30
MIN_MARGIN = 0.06                       # best peak vs best value >= 4 deg away
STILL_DIFF = 6.0                        # mean |diff| (0..255) between two strips = "not moving"
DUP_GAP_S = 0.012                       # frames arriving closer than this to the previous one = burst/repeat


# ---------------------------------------------------------------- geometry
def load_lens(path):
    with open(path) as fh:
        d = json.load(fh)
    return np.array(d["K"], float), np.array(d["D"], float).reshape(4), tuple(d["size"]), \
        float(d.get("theta_max_deg", 51.0))


def fisheye_project(xyz, K, D, theta_max_deg):
    """xyz (N,3) camera rays -> pixel (N,2) with the OpenCV fisheye model; invalid -> nan."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rho = np.hypot(x, y)
    theta = np.arctan2(rho, z)
    t2 = theta * theta
    td = theta * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4)
    s = np.where(rho > 1e-12, td / np.maximum(rho, 1e-12), 1.0)
    u = K[0, 0] * x * s + K[0, 2]
    v = K[1, 1] * y * s + K[1, 2]
    bad = theta > math.radians(theta_max_deg)
    u[bad] = np.nan
    v[bad] = np.nan
    return np.stack([u, v], 1)


def level_to_camera(d, pitch_deg):
    """Levelled rays (x right, y down, z forward horizontal) -> camera rays; camera looks UP by pitch."""
    a = math.radians(pitch_deg)
    ca, sa = math.cos(a), math.sin(a)
    x, y, z = d[..., 0], d[..., 1], d[..., 2]
    return np.stack([x, y * ca + z * sa, -y * sa + z * ca], -1)


def strip_grid():
    az = np.radians(np.arange(AZ[0], AZ[1], 1.0 / PPD))
    el = np.radians(np.arange(EL[1], EL[0], -1.0 / PPD))     # top row = highest elevation
    A, E = np.meshgrid(az, el)
    return np.stack([np.cos(E) * np.sin(A), -np.sin(E), np.cos(E) * np.cos(A)], -1)


def build_strip_map(K, D, theta_max, pitch_deg, src_w, src_h, calib_w):
    """remap tables: strip pixel -> pixel of a src_w x src_h frame (same lens, scaled from calib_w)."""
    sc = src_w / float(calib_w)
    Ks = K.copy()
    Ks[0, :] *= sc
    Ks[1, :] *= sc
    Ks[2, 2] = 1.0
    d = level_to_camera(strip_grid(), pitch_deg)
    h, w = d.shape[:2]
    uv = fisheye_project(d.reshape(-1, 3), Ks, D, theta_max).reshape(h, w, 2)
    valid = np.isfinite(uv).all(-1) & (uv[..., 0] >= 1) & (uv[..., 0] < src_w - 2) \
        & (uv[..., 1] >= 1) & (uv[..., 1] < src_h - 2)
    mx = np.where(valid, uv[..., 0], -1).astype(np.float32)
    my = np.where(valid, uv[..., 1], -1).astype(np.float32)
    return mx, my, valid


class StripMaker:
    def __init__(self, lens_path, pitch_deg, src_w=DECODE_W, src_h=DECODE_H):
        K, D, size, tmax = load_lens(lens_path)
        self.mx, self.my, self.valid = build_strip_map(K, D, tmax, pitch_deg, src_w, src_h, size[0])
        self.src = (src_w, src_h)

    def __call__(self, gray):
        if gray.shape[1] != self.src[0] or gray.shape[0] != self.src[1]:
            gray = cv2.resize(gray, self.src, interpolation=cv2.INTER_AREA)
        s = cv2.remap(gray, self.mx, self.my, cv2.INTER_LINEAR, borderValue=0)
        return s, self.valid


def _features(strip):
    s = cv2.GaussianBlur(strip.astype(np.float32), (0, 0), 1.0)
    gx = cv2.Sobel(s, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(s, cv2.CV_32F, 0, 1, ksize=3)
    return np.sqrt(gx * gx + gy * gy)


def _ncc_at(fa, va, fb, vb, k, dy=0):
    """NCC of after(col c) vs before(col c + k), rows shifted by dy; returns (ncc, overlap_cols)."""
    H, W = fa.shape
    if k >= 0:
        a, av, b, bv = fa[:, :W - k], va[:, :W - k], fb[:, k:], vb[:, k:]
    else:
        a, av, b, bv = fa[:, -k:], va[:, -k:], fb[:, :W + k], vb[:, :W + k]
    if dy:
        if dy > 0:
            a, av, b, bv = a[:-dy], av[:-dy], b[dy:], bv[dy:]
        else:
            a, av, b, bv = a[-dy:], av[-dy:], b[:dy], bv[:dy]
    m = av & bv
    n = int(m.sum())
    cols = int(m.any(0).sum())
    if n < 200:
        return -1.0, cols
    x, y = a[m], b[m]
    x = x - x.mean()
    y = y - y.mean()
    den = math.sqrt(float((x * x).sum()) * float((y * y).sum()))
    return (float((x * y).sum()) / den if den > 0 else -1.0), cols


def yaw_between(before, after, valid, max_shift=MAX_SHIFT):
    """Yaw (deg, + = robot turned right) that maps `before` onto `after` (levelled strips)."""
    fa, fb = _features(after), _features(before)
    # gradients at the edge of the lens' valid area are the same in every frame (they don't move
    # with the scene) and would vote for "no turn": ignore a margin along that edge
    valid = cv2.erode(valid.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    kmax = int(round(max_shift * PPD))
    ks = np.arange(-kmax, kmax + 1)
    scores = np.full(len(ks), -1.0)
    cols = np.zeros(len(ks), int)
    for i, k in enumerate(ks):
        scores[i], cols[i] = _ncc_at(fa, valid, fb, valid, int(k))
    min_cols = MIN_OVERLAP * PPD
    scores[cols < min_cols] = -1.0
    i = int(np.argmax(scores))
    best, k = float(scores[i]), int(ks[i])
    # refine: small vertical wobble and sub-pixel peak
    best_dy = 0
    for dy in (-2, -1, 1, 2):
        s, _ = _ncc_at(fa, valid, fb, valid, k, dy)
        if s > best:
            best, best_dy = s, dy
    sub = 0.0
    if 0 < i < len(ks) - 1 and best_dy == 0:
        l, c, r = scores[i - 1], scores[i], scores[i + 1]
        den = l - 2 * c + r
        if den < 0:
            sub = float(np.clip(0.5 * (l - r) / den, -0.5, 0.5))
    far = np.abs(ks - k) >= 4 * PPD
    second = float(scores[far].max()) if far.any() else -1.0
    deg = (k + sub) / PPD
    overlap = cols[i] / PPD
    ok = best >= MIN_NCC and best - second >= MIN_MARGIN and overlap >= MIN_OVERLAP
    return {"ok": bool(ok), "deg": round(deg, 2), "ncc": round(best, 3), "margin": round(best - second, 3),
            "overlap_deg": round(overlap, 1), "dy_px": best_dy, "method": "strip_ncc"}


# ---------------------------------------------------------------- live frames
class FrameSource:
    """ffmpeg decodes the robot stream to grey frames; keeps the newest ones with arrival times."""

    def __init__(self, url, w=DECODE_W, h=DECODE_H, keep=150):
        self.url, self.w, self.h, self.keep = url, w, h, keep
        self.frames = []          # (t_arrival, gray)
        self.lock = threading.Lock()
        self.proc = None
        self.err = None

    def start(self):
        cmd = ["ffmpeg", "-nostdin", "-loglevel", "error"]
        if self.url.startswith("rtsp"):
            cmd += ["-rtsp_transport", "tcp"]
        cmd += ["-fflags", "nobuffer", "-flags", "low_delay", "-i", self.url, "-an",
                "-vf", f"scale={self.w}:{self.h},format=gray", "-f", "rawvideo", "-pix_fmt", "gray", "pipe:1"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        n = self.w * self.h
        while True:
            buf = self.proc.stdout.read(n)
            if not buf or len(buf) < n:
                self.err = (self.proc.stderr.read() or b"").decode(errors="replace")[-200:]
                return
            g = np.frombuffer(buf, np.uint8).reshape(self.h, self.w)
            if g.std() < 3:           # grey frames before the first keyframe
                continue
            with self.lock:
                self.frames.append((time.monotonic(), g))
                del self.frames[:-self.keep]

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def since(self, t):
        with self.lock:
            return [f for f in self.frames if f[0] > t]

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


class Service:
    def __init__(self, source_factory=FrameSource, clock=time.monotonic, sleep=time.sleep):
        self.src = None
        self.mk = None
        self.store = {}
        self.next_id = 1
        self.source_factory, self.clock, self.sleep = source_factory, clock, sleep

    def _keep(self, t, strip):
        i = self.next_id
        self.next_id += 1
        self.store[i] = (t, strip)
        for old in sorted(self.store)[:-40]:
            del self.store[old]
        return i

    def start(self, url, lens, pitch_deg=15.0, wait=10.0):
        if self.src is None or not self.src.alive() or self.src.url != url:
            if self.src:
                self.src.stop()
            self.src = self.source_factory(url)
            self.src.start()
        if self.mk is None:
            self.mk = StripMaker(lens, pitch_deg, self.src.w, self.src.h)
        end = self.clock() + wait
        while self.clock() < end:
            if self.src.since(self.clock() - 1.0):
                return {"ok": True}
            if not self.src.alive():
                break
            self.sleep(0.1)
        return {"ok": False, "reason": "no frames from the robot stream" + (f": {self.src.err}" if self.src.err else "")}

    def _distinct(self, after, gap=DUP_GAP_S):
        """Frames newer than `after` that are new evidence: decoder repeats (byte-identical to the previous
        kept frame) and burst frames after a stall (arriving < gap after it) are dropped - a frozen
        stream must not look like a robot at rest."""
        return distinct_frames(self.src.since(after), lambda a, b: np.array_equal(a, b), gap)

    def _is_still(self, a, b, v):
        return float(np.abs(a.astype(np.int16) - b.astype(np.int16))[v].mean()) < STILL_DIFF

    def still(self, after, max_wait=3.0, hold_s=0.0, hold_n=2):
        """A still frame newer than `after`. hold_s == 0: the first pair of consecutive distinct frames
        that do not differ. hold_s > 0: the NEWEST frames must form a still run of >= hold_n distinct
        frames spanning >= hold_s (the robot has been at rest for a while - no leftover motion)."""
        end = self.clock() + max_wait
        cache = {}
        v = self.mk.valid
        while self.clock() < end:
            fr = self._distinct(after)
            strips = [(t, self._strip_cached(cache, t, g)) for t, g in fr]
            if hold_s <= 0:
                for (t0, s0), (t1, s1) in zip(strips, strips[1:]):
                    if self._is_still(s0, s1, v):
                        return {"ok": True, "id": self._keep(t1, s1), "t": t1}
            elif strips:
                run = [strips[-1]]
                for k in range(len(strips) - 2, -1, -1):
                    if self._is_still(strips[k][1], run[0][1], v):
                        run.insert(0, strips[k])
                    else:
                        break
                if len(run) >= hold_n and run[-1][0] - run[0][0] >= hold_s:
                    t1, s1 = run[-1]
                    return {"ok": True, "id": self._keep(t1, s1), "t": t1, "still_s": round(t1 - run[0][0], 3)}
            self.sleep(0.05)
        n = len(self._distinct(after)) if self.src else 0
        return {"ok": False, "reason": ("picture not still" if hold_s > 0 else "no still frame") if n
                else "no fresh frames"}

    def _strip_cached(self, cache, t, g):
        if t not in cache:
            cache[t] = self.mk(g)[0]
        return cache[t]

    def settle(self, before, t_start, t_end, cfg):
        """After a pulse: wait until the picture has visibly changed vs the before-frame and is still
        again (or, if nothing changed, until no_motion_s), never accepting a frame older than END.
        Only distinct new frames count (cfg distinct_only); a result that is implausibly small for the
        pulse kind (cfg plaus_min_deg[kind]) keeps waiting until END + plaus_hold_s.
        Returns the settled frame + measured stream latency."""
        if before not in self.store:
            return {"ok": False, "reason": "unknown frame id"}
        bt, bstrip = self.store[before]
        v = self.mk.valid
        cache = {}
        checked = {}
        kind = cfg.get("kind")
        deadline = t_end + float(cfg["settle_max_s"]) + 0.5
        distinct = cfg.get("distinct_only", True)
        while True:
            now = self.clock()
            fr = self._distinct(t_start, float(cfg.get("dup_gap_s", DUP_GAP_S))) if distinct \
                else self.src.since(t_start)
            meta, prev = [], None
            for t, g in fr:
                s = self._strip_cached(cache, t, g)
                changed = not self._is_still(bstrip, s, v)
                still = prev is not None and self._is_still(prev, s, v)
                meta.append((t, changed, still))
                prev = s
            state, idx, info = settle_pick(meta, now, t_start, t_end, cfg)
            if state == "ok" and info["moved"] and kind:
                t = meta[idx][0]
                if t not in checked:
                    r = yaw_between(bstrip, cache[t], v)
                    checked[t] = (not r.get("ok")) or plausible(r["deg"], kind, t, t_end, cfg)
                if not checked[t]:
                    state = "wait"          # e.g. a full pulse reading +5 deg: the motion is still coming
            if state == "ok":
                t = meta[idx][0]
                i = self._keep(t, cache[t])
                tc = info["t_change"]
                return {"ok": True, "id": i, "t": t, "moved": info["moved"],
                        "latency_s": None if tc is None else round(tc - t_start, 3),
                        "latency_end_s": None if tc is None else round(tc - t_end, 3),
                        "settle_s": round(t - t_end, 3), "frames": len(meta)}
            if state == "timeout" or now > deadline:
                return {"ok": False, "reason": "picture did not settle" if meta else "no fresh frames",
                        "moved": info["moved"], "frames": len(meta)}
            self.sleep(0.05)

    def yaw(self, a, b):
        if a not in self.store or b not in self.store:
            return {"ok": False, "reason": "unknown frame id"}
        return yaw_between(self.store[a][1], self.store[b][1], self.mk.valid)

    def chain(self, a, b, t0, t1, step=2):
        """Fallback for turns too large for one registration: add up the yaw between consecutive
        (sharp enough) frames recorded during the motion, ending at still frame b."""
        if a not in self.store or b not in self.store:
            return {"ok": False, "reason": "unknown frame id"}
        mids = [self.mk(g)[0] for t, g in self.src.since(t0) if t <= t1][::step]
        seq = [self.store[a][1]] + mids + [self.store[b][1]]
        total, used = 0.0, 0
        prev = seq[0]
        for nxt in seq[1:]:
            r = yaw_between(prev, nxt, self.mk.valid, max_shift=45.0)
            if r["ok"]:
                total += r["deg"]
                prev = nxt
                used += 1
        direct = yaw_between(prev, seq[-1], self.mk.valid)
        ok = used > 0 and prev is seq[-1]
        return {"ok": bool(ok), "deg": round(total, 2), "steps": used, "method": "chain",
                "tail_ok": direct["ok"]}

    def handle(self, req):
        c = req.get("cmd")
        if c == "start":
            return self.start(req["url"], req["lens"], float(req.get("pitch_deg", 15.0)),
                              float(req.get("wait", 10.0)))
        if self.src is None:
            return {"ok": False, "reason": "not started"}
        if c == "still":
            return self.still(float(req["after"]), float(req.get("max_wait", 3.0)),
                              float(req.get("hold_s", 0.0)), int(req.get("hold_n", 2)))
        if c == "settle":
            return self.settle(int(req["before"]), float(req["t_start"]), float(req["t_end"]), req)
        if c == "yaw":
            return self.yaw(int(req["a"]), int(req["b"]))
        if c == "chain":
            return self.chain(int(req["a"]), int(req["b"]), float(req["t0"]), float(req["t1"]))
        if c == "stop":
            self.src.stop()
            self.src = None
            return {"ok": True}
        return {"ok": False, "reason": f"unknown cmd {c!r}"}


def serve():
    svc = Service()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            out = svc.handle(json.loads(line))
        except Exception as e:  # never die on one bad request
            out = {"ok": False, "reason": f"{type(e).__name__}: {e}"[:300]}
        sys.stdout.write(json.dumps(out) + "\n")
        sys.stdout.flush()
    if svc.src:
        svc.src.stop()


if __name__ == "__main__":
    if "--serve" in sys.argv:
        serve()
    else:
        print(__doc__)
