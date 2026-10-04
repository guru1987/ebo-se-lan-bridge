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
    "blip_step": 0.02,          # a lost blip (< lost_deg) lengthens that side's blip by this much
    "blip_max": 0.12,
    "lost_deg": 2.0,
    "settle_s": 0.45,           # after the last motor frame: run-on of a full turn + stream latency
    "still_wait_s": 3.0,
    "wrong_way_deg": 4.0,       # a pulse that turns the other way by more than this aborts the turn
    "direct_max_deg": 65.0,     # re-measure the total against the start frame while within this
    "direct_agree_deg": 25.0,   # ...and accept it only if it roughly agrees with the summed pulses
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


class TurnError(Exception):
    pass


def run_turn(target, tol, max_pulses, pulse, vision, cfg=None, log=print):
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
                "uncertain": uncertain, "reason": reason}

    ref = vision.still(time.monotonic() - 0.05, cfg["still_wait_s"])
    if not ref.get("ok"):
        return result(False, "camera: " + ref.get("reason", "no still frame before turning"))
    ref_id = start_id = ref["id"]
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
                m, method = {"deg": sign * expect}, "nominal"
                uncertain = True
        deg = float(m["deg"])
        p = {"kind": kind, "rx": sign * cfg["rx"], "dur": secs, "measured_deg": round(deg, 1),
             "method": method}
        if method == "strip_ncc":
            p["ncc"] = m.get("ncc")
        pulses.append(p)
        log(f"[turn] pulse {n + 1}: {kind} {side} {secs:.2f}s -> {deg:+.1f} deg ({method})")
        if deg * sign < -cfg["wrong_way_deg"]:
            total += deg
            return result(False, f"pulse turned the wrong way ({deg:+.1f} deg) - stopped")
        total += deg
        ref_id = st["id"]
        if abs(total) <= cfg["direct_max_deg"]:
            # while the start view still overlaps, measure the TOTAL directly against the first frame:
            # no accumulated error from earlier pulses (nor from a chained / nominal one)
            d = vision.yaw(start_id, st["id"])
            if d.get("ok") and abs(float(d["deg"]) - total) <= cfg["direct_agree_deg"]:
                total = float(d["deg"])
                p["total_direct_deg"] = round(total, 1)
                uncertain = False
        mag = abs(deg)
        if mag < cfg["lost_deg"]:
            if kind == "blip":
                dur["blip"][side] = min(cfg["blip_max"], round(dur["blip"][side] + cfg["blip_step"], 3))
        elif method != "nominal":
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

    def call(self, obj, timeout=15.0):
        with self.lock:
            self.last = time.monotonic()
            if self.proc is None or self.proc.poll() is not None:
                self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                r = self._req({"cmd": "start", "url": self.url, "lens": self.lens,
                               "pitch_deg": self.pitch}, 20)
                if not r.get("ok"):
                    raise TurnError(r.get("reason", "vision helper failed to start"))
            return self._req(obj, timeout)

    def ensure_started(self):
        r = self.call({"cmd": "still", "after": time.monotonic() - 1.0, "max_wait": 0.1}, 25)
        return r

    def still(self, after, max_wait):
        return self.call({"cmd": "still", "after": after, "max_wait": max_wait}, max_wait + 10)

    def yaw(self, a, b):
        return self.call({"cmd": "yaw", "a": a, "b": b}, 20)

    def chain(self, a, b, t0, t1):
        return self.call({"cmd": "chain", "a": a, "b": b, "t0": t0, "t1": t1}, 60)
