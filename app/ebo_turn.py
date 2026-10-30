"""POST /turn: turn the EBO by a measured angle (closed loop).

The EBO firmware cannot turn by an arbitrary angle on command. Measured from above at 446 fps
(tools/turncal, runs 1-8): a turn pulse either ends as a short "blip" (~7-27 deg, never full speed)
or as a full turn (right ~50 deg +-10, left ~61 deg +-4 on bare floor), pulses of 0.12-0.18 s are a
coin flip between the two, and very short pulses are sometimes lost. Timing alone therefore cannot
hit an angle.

So /turn closes the loop with the robot's own camera: grab a still frame, send ONE pulse, wait until
the picture is still again, measure the yaw between the two still frames (tools/ebo_turn_vision.py,
~1 deg while the views overlap), add it up, and choose the next pulse from what is left:
  remaining >= the side's full-turn threshold -> full pulse; otherwise a blip;
  stop when within tolerance, or when no pulse is expected to improve the error, or after max_pulses.
Nominal pulse sizes adapt to what this run actually measures. If a single registration fails (turn
too large for the overlap), the frames recorded during the motion are chained; if that fails too,
the pulse counts with its nominal size and the result is marked "uncertain".
Every pulse goes through the bridge's normal move path (dock guard, warm-up from idle, 20 ms frames).
Sign: + = right (clockwise seen from above), like rx.

This module has no numpy/OpenCV dependency: the vision runs in a helper process (VisionClient).
"""
import json
import math
import os
import subprocess
import threading
import time

DEFAULTS = {
    "rx": 0.6,
    # pulse seconds per kind and side; nominal |deg| they usually give (adapted during a run)
    "dur": {"blip": {"right": 0.10, "left": 0.10}, "full": {"right": 0.20, "left": 0.20}},
    "nominal": {"blip": {"right": 15.0, "left": 15.0}, "full": {"right": 50.0, "left": 61.0}},
    "blip_min": {"right": 7.0, "left": 7.0},   # smallest blip seen (deg); a blip is only tried if
                                               # some outcome could land within the tolerance
    "full_min_frac": 0.8,       # use a full pulse when remaining >= 0.8 x the side's nominal full turn
    "blip_step": 0.02,          # a lost blip (< lost_deg) lengthens that side's blip by this much,
                                # a blip that became a full turn shortens it by this much
    "blip_max": 0.14,
    "blip_min_dur": 0.06,       # Juraj 2026-10-04 (20 ms frames + warm-up): left 0.06 and 0.13 s gave
                                # small, usable steps -> short left blips are allowed and adapted
    "lost_deg": 2.0,
    "settle_s": 0.45,           # legacy fixed wait (settle_mode "still" only)
    "still_wait_s": 3.0,
    # settle detection (live 2026-10-04: the robot stream lags 0.3-1.5 s after a wake, so a fixed wait
    # measured frames from BEFORE the motion and every pulse was counted one pulse late)
    "settle_mode": "settle",    # "settle" (wait until the picture moved and is still again) | "still"
    # live closure test 2026-10-04 (4 x 90 deg): a full pulse "settled" 0.50 s after END at +5.6 deg -
    # leftover motion of the previous /turn plus a frozen stretch of the stream looked like a still
    # picture before the real motion had arrived. Hence: longer minimum wait, longer stillness, only
    # distinct new frames count, an implausibly small result keeps waiting, and the start frame must be
    # still for a while.
    "settle_min_s": 0.8,        # never accept an after-frame earlier than END + this
    "settle_max_s": 3.5,        # give up (uncertain) when not settled by END + this
    "no_motion_s": 1.5,         # no change seen until END + this -> the pulse was lost (0 deg) ...
    "no_motion_full_s": 2.5,    # ... for a FULL pulse (rarely lost; the stream can lag > 1.5 s)
    "stable_n": 5,              # settled = >= stable_n consecutive still DISTINCT frames ...
    "stable_s": 0.5,            # ... spanning >= stable_s, all newer than END and than the first change
    "distinct_only": True,      # frames that repeat the previous one (same content, or arriving < dup_gap_s
    "dup_gap_s": 0.012,         # after it: decoder repeats / bursts after a stall) are no evidence of rest
    "plaus_min_deg": {"full": 30.0, "blip": 3.0},   # a pulse that reads smaller than this is "not
    "plaus_hold_s": 1.5,        # settled yet" unless it stays that small until END + plaus_hold_s
    "start_still_s": 0.5,       # before the first pulse: the picture must be still this long ...
    "start_still_n": 5,         # ... over at least this many distinct frames (no leftover motion)
    "start_wait_s": 5.0,        # ... waiting at most this long for it
    "high_latency_s": 1.0,      # log a warning when the stream lags more than this (and keep going)
    "wrong_way_deg": 4.0,       # a pulse that turns the other way by more than this aborts the turn
    "direct_max_deg": 65.0,     # re-measure the total against the start frame while within this
    "direct_agree_deg": 25.0,
    "parallax": True,           # correct each measured yaw for the camera sitting cam_offset_m in front of
    "cam_offset_m": 0.048,      # the turning centre (near objects shift more than the robot turns)
    "strip_half_deg": 50.0,     # the vision strip spans +/- this azimuth (tools/ebo_turn_vision.py AZ)
    "range_min_m": 0.25,        # clamp depth-sector ranges (nearest obstacle per sector) to this ...
    "range_far_m": 10.0,        # ... and treat non-obstacle sectors as this far (~pure rotation)
    "lat_unknown_s": 2.0,       # stream latency assumed before any was measured (fresh bridge start)   # ...and accept it only if it roughly agrees with the summed pulses
}


def load_config(path=None):
    cfg = json.loads(json.dumps(DEFAULTS))
    path = path or os.environ.get("EBO_TURN_CONFIG")
    if path and os.path.exists(path):
        user = json.load(open(path))
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                for k2, v2 in v.items():
                    if isinstance(v2, dict):
                        cfg[k].setdefault(k2, {}).update(v2)
                    else:
                        cfg[k][k2] = v2
            else:
                cfg[k] = v
    return cfg


def _bearing_after(beta, d, theta, r):
    """Point at bearing beta (rad, + = right) and range d (m) from a camera r m in front of the turning
    centre; the robot turns by theta (rad, + = right). -> the point's bearing in the new camera view."""
    px, py = d * math.sin(beta), r + d * math.cos(beta)
    c, s_ = math.cos(theta), math.sin(theta)
    vx, vy = px - r * s_, py - r * c
    return math.atan2(vx * c - vy * s_, vx * s_ + vy * c)


def _range_at(beta_deg, sectors, cfg):
    best = min(sectors, key=lambda x: abs(x[0] - beta_deg))
    m, kind = float(best[1]), (best[2] if len(best) > 2 else "obstacle")
    if kind != "obstacle":
        return cfg["range_far_m"]
    return min(cfg["range_far_m"], max(cfg["range_min_m"], m))


def apparent_yaw(theta_deg, sectors, cfg):
    """What the strip registration reads for a true turn theta_deg: mean column shift over the strip
    overlap, each column shifted by the exact 2-D geometry for its depth-sector range."""
    half, r = cfg["strip_half_deg"], cfg["cam_offset_m"]
    lo, hi = (-half + theta_deg, half) if theta_deg >= 0 else (-half, half + theta_deg)
    if hi - lo < 2.0 or not sectors:
        return theta_deg
    n, th, acc = 41, math.radians(theta_deg), 0.0
    for i in range(n):
        b = lo + (hi - lo) * i / (n - 1)
        br = math.radians(b)
        acc += b - math.degrees(_bearing_after(br, _range_at(b, sectors, cfg), th, r))
    return acc / n


def parallax_true_yaw(meas_deg, sectors, cfg):
    """Invert apparent_yaw: the true turn that reads as meas_deg (bisection; parallax only enlarges)."""
    if not cfg.get("parallax") or not sectors or abs(meas_deg) < 0.5:
        return meas_deg
    lo, hi = 0.0, meas_deg
    if (apparent_yaw(hi, sectors, cfg) - meas_deg) * (1 if meas_deg > 0 else -1) < 0:
        return meas_deg            # outside the model's range: leave it
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if (apparent_yaw(mid, sectors, cfg) - meas_deg) * (1 if meas_deg > 0 else -1) < 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def settle_pick(frames, now, t_start, t_end, cfg):
    """Decide whether the picture after a pulse has settled.

    frames: list of (t, changed, still) sorted by t, all with t > t_start, where `changed` = the frame
    differs from the before-frame (the robot has visibly turned) and `still` = it does not differ from
    the previous frame. Returns (state, index, info): state "ok" -> frames[index] is the after-frame;
    "wait" -> ask again later; "timeout" -> give up. info has t_change (first changed frame or None)
    and moved (bool)."""
    t_change = next((t for t, ch, _ in frames if ch), None)
    info = {"t_change": t_change, "moved": t_change is not None}
    if now < t_end + cfg["settle_min_s"]:
        return "wait", None, info
    t_from = max(t_end, t_change) if t_change is not None else t_end
    cand = [i for i, (t, _, _) in enumerate(frames) if t > t_from]
    # newest run of consecutive still frames
    run = []
    for i in reversed(cand):
        if frames[i][2]:
            run.insert(0, i)
        else:
            break
    if len(run) >= cfg["stable_n"] and frames[run[-1]][0] - frames[run[0]][0] >= cfg["stable_s"]:
        if t_change is not None:
            return "ok", run[-1], info
        if now >= t_end + cfg["no_motion_s"]:
            return "ok", run[-1], info     # nothing moved: the pulse was lost
    if now >= t_end + cfg["settle_max_s"]:
        return "timeout", None, info
    return "wait", None, info


def plausible(deg, kind, t_cand, t_end, cfg):
    """Is a settled measurement believable? A full pulse that reads < plaus_min_deg["full"] (or a blip
    < plaus_min_deg["blip"]) most likely means the motion has not reached the stream yet - keep waiting
    unless the picture has stayed like that until END + plaus_hold_s (then it really was small/lost)."""
    lim = float((cfg.get("plaus_min_deg") or {}).get(kind, 0.0))
    if abs(deg) >= lim:
        return True
    return t_cand - t_end >= float(cfg.get("plaus_hold_s", 0.0))


def distinct_frames(frames, same, gap):
    """Drop frames that are no new evidence: arriving < gap s after the previously kept one (burst after a
    stall) or `same` as it (decoder repeat). frames: [(t, payload)]; same(a, b) -> bool on payloads."""
    out = []
    for t, x in frames:
        if out and (t - out[-1][0] < gap or same(out[-1][1], x)):
            continue
        out.append((t, x))
    return out


# Stream latency seen recently (seconds from pulse END to the first visible change), kept across /turn calls:
# the no-motion and plausibility windows scale with it, so a stream that lags 1.5 s is not mistaken for a
# lost pulse whose motion then leaks into the next measurement.
RECENT_LAT = []


def note_latency(lat_end):
    if lat_end is not None and lat_end >= 0:
        RECENT_LAT.append(float(lat_end))
        del RECENT_LAT[:-10]


def latency_estimate():
    return max(RECENT_LAT[-5:]) if RECENT_LAT else 0.0


class TurnError(Exception):
    pass


def run_turn(target, tol, max_pulses, pulse, vision, cfg=None, log=print, depth=None):
    """pulse(rx, seconds) -> (ok, info, t_end) where t_end = time.monotonic() after the last frame.
    vision: object with still(after, max_wait) / yaw(a, b) / chain(a, b, t0, t1) returning dicts.
    Returns the /turn result dict."""
    cfg = cfg or load_config()
    nom = json.loads(json.dumps(cfg["nominal"]))
    bmin = dict(cfg["blip_min"])
    dur = json.loads(json.dumps(cfg["dur"]))
    pulses, total, uncertain = [], 0.0, False

    def result(ok, reason):
        return {"ok": bool(ok), "target_deg": target, "turned_deg": round(total, 1),
                "error_deg": round(total - target, 1), "tol_deg": tol, "pulses": pulses,
                "uncertain": uncertain, "reason": reason, "blip_dur": dict(dur["blip"])}

    ref = vision.still(time.monotonic(), cfg["start_wait_s"], cfg["start_still_s"], cfg["start_still_n"])
    if not ref.get("ok"):
        return result(False, "camera: " + ref.get("reason", "no still frame before turning"))
    ref_id = start_id = ref["id"]

    def sectors_now():
        if depth is None or not cfg.get("parallax"):
            return None
        try:
            return depth()
        except Exception as e:      # depth is optional: no correction rather than no turn
            log(f"[turn] depth unavailable ({e}); no parallax correction")
            return None

    start_sec = ref_sec = sectors_now()
    for n in range(max_pulses):
        rem = target - total
        if abs(rem) <= tol:
            return result(True, "within tolerance")
        side = "right" if rem > 0 else "left"
        sign = 1 if rem > 0 else -1
        kind = "full" if abs(rem) >= cfg["full_min_frac"] * nom["full"][side] else "blip"
        expect = nom[kind][side]
        if kind == "blip" and abs(rem) + tol < bmin[side]:
            # even the smallest blip seen would overshoot beyond the tolerance
            return result(False, f"cannot refine further (smallest {side} pulse ~{bmin[side]:.0f} deg)")
        secs = dur[kind][side]
        t_start = time.monotonic()
        ok, info, t_end = pulse(sign * cfg["rx"], secs)
        if not ok:
            return result(False, "pulse refused: " + str(info.get("reason") or info))
        if cfg.get("settle_mode", "settle") == "settle" and hasattr(vision, "settle"):
            fresh = not RECENT_LAT
            lat = latency_estimate() if not fresh else cfg["lat_unknown_s"]
            ckind = dict(cfg, kind=kind)
            if fresh:
                # nothing measured yet (fresh start): do not accept an after-frame before the stream
                # can possibly show the motion
                ckind["settle_min_s"] = max(cfg["settle_min_s"], lat)
            nm = cfg.get("no_motion_full_s", cfg["no_motion_s"]) if kind == "full" else cfg["no_motion_s"]
            ckind["no_motion_s"] = max(nm, lat + (1.0 if kind == "full" else 0.7))
            ckind["plaus_hold_s"] = max(cfg["plaus_hold_s"], lat + 0.7)
            ckind["settle_max_s"] = max(cfg["settle_max_s"], ckind["no_motion_s"] + 1.2)
            st = vision.settle(ref_id, t_start, t_end, ckind)
            if st.get("ok") and st.get("moved"):
                note_latency(st.get("latency_end_s"))
            elif st.get("ok") and st.get("moved") is False:
                # looks lost: confirm with a longer rest window (a late motion would still show up here)
                hold = max(cfg["start_still_s"], lat + 0.5)
                st2 = vision.still(time.monotonic(), hold + 3.0, hold, cfg["start_still_n"])
                if st2.get("ok"):
                    m2 = vision.yaw(ref_id, st2["id"])
                    if m2.get("ok") and abs(float(m2["deg"])) >= cfg["lost_deg"]:
                        st = dict(st2, moved=True, late=True, settle_s=round(st2["t"] - t_end, 3))
                        log(f"[turn] pulse {n + 1}: motion arrived late ({float(m2['deg']):+.1f} deg)")
        else:
            st = vision.still(t_end + cfg["settle_s"], cfg["still_wait_s"])
        if not st.get("ok"):
            pulses.append({"kind": kind, "rx": sign * cfg["rx"], "dur": secs, "measured_deg": None,
                           "method": "none"})
            return result(False, "camera: " + st.get("reason", "no still frame after the pulse"))
        m = vision.yaw(ref_id, st["id"])
        method = "strip_ncc"
        if not m.get("ok"):
            c = vision.chain(ref_id, st["id"], t_start, st["t"])
            if c.get("ok"):
                m, method = c, "chain"
                uncertain = True       # chained frames tend to read short; corrected below if possible
            else:
                # unmeasured: wait for a long rest (late motion included) and measure again
                hold = max(cfg["start_still_s"], (latency_estimate() or cfg["lat_unknown_s"]) + 0.5)
                st2 = vision.still(time.monotonic(), hold + 3.0, hold, cfg["start_still_n"])
                m2 = vision.yaw(ref_id, st2["id"]) if st2.get("ok") else {"ok": False}
                if m2.get("ok"):
                    m, method, st = m2, "strip_ncc_late", st2
                else:
                    m, method = {"deg": sign * expect}, "nominal"
                    uncertain = True
        deg = float(m["deg"])
        raw = deg
        if method != "nominal":
            deg = parallax_true_yaw(raw, ref_sec, cfg)
        p = {"kind": kind, "rx": sign * cfg["rx"], "dur": secs, "measured_deg": round(deg, 1),
             "method": method}
        if method == "strip_ncc":
            p["ncc"] = m.get("ncc")
        if method != "nominal" and abs(deg - raw) >= 0.05:
            p["measured_raw_deg"] = round(raw, 1)
        for k in ("latency_s", "latency_end_s", "settle_s", "moved"):
            if st.get(k) is not None:
                p[k] = st[k]
        pulses.append(p)
        lat = (f" lat {st['latency_s']:.2f}s" if st.get("latency_s") is not None else " no motion") \
            if "moved" in st else ""
        set_s = f" settled {st['settle_s']:.2f}s after END" if st.get("settle_s") is not None else ""
        log(f"[turn] pulse {n + 1}: {kind} {side} {secs:.2f}s -> {deg:+.1f} deg ({method}){lat}{set_s}")
        if st.get("latency_end_s") is not None and st["latency_end_s"] > cfg["high_latency_s"]:
            log(f"[turn] note: high stream latency {st['latency_end_s']:.2f} s after END (continuing)")
        if deg * sign < -cfg["wrong_way_deg"]:
            # re-measure once on a fresh still frame before believing it (a late frame can fake this)
            st2 = vision.still(time.monotonic(), cfg["start_wait_s"], cfg["start_still_s"],
                               cfg["start_still_n"])
            m2 = vision.yaw(ref_id, st2["id"]) if st2.get("ok") else {"ok": False}
            if m2.get("ok") and float(m2["deg"]) * sign >= -cfg["wrong_way_deg"]:
                deg = float(m2["deg"])
                p["measured_deg"], p["remeasured"] = round(deg, 1), True
                st = st2
                log(f"[turn] pulse {n + 1}: re-measured {deg:+.1f} deg")
            else:
                total += deg
                return result(False, f"pulse turned the wrong way ({deg:+.1f} deg) - stopped")
        total += deg
        ref_id = st["id"]
        ref_sec = sectors_now()
        if abs(total) <= cfg["direct_max_deg"]:
            # while the start view still overlaps, measure the TOTAL directly against the first frame:
            # no accumulated error from earlier pulses (nor from a chained / nominal one)
            d = vision.yaw(start_id, st["id"])
            dd = parallax_true_yaw(float(d["deg"]), start_sec, cfg) if d.get("ok") else None
            if dd is not None and abs(dd - total) <= cfg["direct_agree_deg"]:
                total = dd
                p["total_direct_deg"] = round(total, 1)
                uncertain = False
        mag = abs(deg)
        if mag < cfg["lost_deg"]:
            if kind == "blip":
                dur["blip"][side] = min(cfg["blip_max"], round(dur["blip"][side] + cfg["blip_step"], 3))
        elif kind == "blip" and method != "nominal" and \
                mag >= 0.5 * (nom["blip"][side] + nom["full"][side]):
            # the blip came out as a full turn (coin-flip region): next blip on this side shorter
            dur["blip"][side] = max(cfg["blip_min_dur"], round(dur["blip"][side] - cfg["blip_step"], 3))
        if mag >= cfg["lost_deg"] and method != "nominal":
            # learn what this run's pulses really do (a blip that became a full turn says nothing
            # about blips: classify by size)
            k2 = "full" if mag >= 0.5 * (nom["blip"][side] + nom["full"][side]) else "blip"
            nom[k2][side] = round(0.5 * nom[k2][side] + 0.5 * mag, 1)
            if k2 == "blip":
                bmin[side] = min(bmin[side], mag)
    rem = target - total
    return result(abs(rem) <= tol, "within tolerance" if abs(rem) <= tol else f"max_pulses ({max_pulses}) reached")


class VisionClient:
    """Runs tools/ebo_turn_vision.py --serve in a Python that has numpy + OpenCV; JSON lines over a pipe.
    Started on first use, stopped after `idle_s` without requests."""

    def __init__(self, python, script, url, lens, pitch_deg, idle_s=60.0):
        self.cmd = [python, script, "--serve"]
        self.url, self.lens, self.pitch = url, lens, pitch_deg
        self.idle_s = idle_s
        self.proc = None
        self.lock = threading.Lock()
        self.last = 0.0
        threading.Thread(target=self._idle_loop, daemon=True).start()

    def _idle_loop(self):
        while True:
            time.sleep(5)
            with self.lock:
                if self.proc and self.proc.poll() is None and time.monotonic() - self.last > self.idle_s:
                    self._req({"cmd": "stop"}, 5)
                    self.proc.stdin.close()
                    self.proc = None

    def _req(self, obj, timeout):
        self.proc.stdin.write((json.dumps(obj) + "\n").encode())
        self.proc.stdin.flush()
        out = {}
        done = threading.Event()

        def rd():
            line = self.proc.stdout.readline()
            if line:
                out.update(json.loads(line))
            done.set()
        threading.Thread(target=rd, daemon=True).start()
        if not done.wait(timeout):
            self.proc.kill()
            self.proc = None
            raise TurnError(f"vision helper timed out on {obj.get('cmd')}")
        if not out:
            raise TurnError("vision helper exited")
        return out

    # F55 (2026-10-29): "start" goes out on EVERY call, not only when a new helper is spawned. After the
    # video idle-pause the helper process stayed alive with a dead/stalled decoder and every later /turn
    # failed "no fresh frames". The helper's start is a no-op while its decoder is alive and delivering
    # frames (returns at once); otherwise it (re)starts the decoder and waits for the first frames. If
    # that fails (e.g. the RTSP path has no publisher yet right after video re-enable) the helper is
    # killed and respawned, retrying until start_retry_s has passed.
    start_retry_s = 8.0
    start_wait_s = 3.0

    def _kill(self):
        if self.proc is not None:
            try:
                self.proc.kill()
            except Exception:
                pass
            self.proc = None

    def _ensure_frames(self):
        deadline = time.monotonic() + self.start_retry_s
        reason = "vision helper failed to start"
        while True:
            if self.proc is None or self.proc.poll() is not None:
                self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            try:
                r = self._req({"cmd": "start", "url": self.url, "lens": self.lens,
                               "pitch_deg": self.pitch, "wait": self.start_wait_s}, self.start_wait_s + 20)
            except TurnError as e:
                r = {"ok": False, "reason": str(e)}
            if r.get("ok"):
                return
            reason = r.get("reason") or reason
            self._kill()                  # fresh helper + fresh decoder on the next try
            if time.monotonic() >= deadline:
                raise TurnError(reason)
            time.sleep(0.5)

    def call(self, obj, timeout=15.0):
        with self.lock:
            self.last = time.monotonic()
            self._ensure_frames()
            return self._req(obj, timeout)

    def ensure_started(self):
        r = self.call({"cmd": "still", "after": time.monotonic() - 1.0, "max_wait": 0.1}, 25)
        return r

    def still(self, after, max_wait, hold_s=0.0, hold_n=2):
        return self.call({"cmd": "still", "after": after, "max_wait": max_wait, "hold_s": hold_s,
                          "hold_n": hold_n}, max_wait + 10)

    def settle(self, before_id, t_start, t_end, cfg):
        keys = ("settle_min_s", "settle_max_s", "no_motion_s", "stable_n", "stable_s", "distinct_only",
                "dup_gap_s", "plaus_min_deg", "plaus_hold_s", "kind")
        req = {"cmd": "settle", "before": before_id, "t_start": t_start, "t_end": t_end}
        req.update({k: cfg[k] for k in keys})
        return self.call(req, cfg["settle_max_s"] + 15)

    def yaw(self, a, b):
        return self.call({"cmd": "yaw", "a": a, "b": b}, 20)

    def chain(self, a, b, t0, t1):
        return self.call({"cmd": "chain", "a": a, "b": b, "t0": t0, "t1": t1}, 60)
