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

    def still(self, after, max_wait=3.0):
        """First pair of consecutive frames newer than `after` that do not differ (robot at rest)."""
        end = self.clock() + max_wait
        while self.clock() < end:
            fr = self.src.since(after)
            for (t0, g0), (t1, g1) in zip(fr, fr[1:]):
                s0, v = self.mk(g0)
                s1, _ = self.mk(g1)
                if float(np.abs(s1.astype(np.int16) - s0.astype(np.int16))[v].mean()) < STILL_DIFF:
                    return {"ok": True, "id": self._keep(t1, s1), "t": t1}
            self.sleep(0.05)
        n = len(self.src.since(after)) if self.src else 0
        return {"ok": False, "reason": "no still frame" if n else "no fresh frames"}

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
            return self.still(float(req["after"]), float(req.get("max_wait", 3.0)))
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
