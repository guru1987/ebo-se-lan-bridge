#!/usr/bin/env python3
"""Merged, human-readable timeline of the bridge's always-on command log (app/ebo_cmdlog.py).

  ebolog.py --since 16:39:40 --until 16:40:10          # today, local time
  ebolog.py --since "2026-10-04 16:39" --no-rx         # only HTTP + TX
  ebolog.py --last 120                                 # the last 120 s
Identical consecutive TX frames (warm-up, motion streams) are collapsed to one line "×N" unless --raw.
"""
import argparse
import datetime as dt
import glob
import json
import os
import sys
import time

DEF_DIR = os.path.expanduser(os.environ.get("EBO_LOG_DIR", "~/ebo_bridge/runtime/logs"))


def parse_t(s, today):
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        pass
    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%H:%M:%S", "%H:%M"):
        try:
            t = dt.datetime.strptime(s, fmt)
            if fmt.startswith("%H"):
                t = t.replace(year=today.year, month=today.month, day=today.day)
            return t.timestamp()
        except ValueError:
            continue
    sys.exit(f"cannot parse time {s!r}")


def load(d, stream, t0, t1):
    files = sorted(glob.glob(os.path.join(d, f"{stream}.jsonl*")),
                   key=lambda p: -int(p.rsplit(".", 1)[1]) if p[-1].isdigit() else 0)
    out = []
    for p in files:
        with open(p, errors="replace") as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if t0 <= r.get("ts", 0) <= t1:
                    r["_s"] = stream
                    out.append(r)
    return out


def fmt_ts(ts):
    return dt.datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]


def tx_key(r):
    return tuple((k, r.get(k)) for k in ("kind", "msg", "lx", "ly", "rx", "ry", "buttons", "command",
                                         "param", "value", "io_type", "payload", "origin", "req"))


def describe(r):
    s = r["_s"]
    if s == "http":
        return (f"HTTP {r.get('client')} {r.get('user') or '-'} {r.get('method')} {r.get('path')} "
                f"{(r.get('body') or '')[:120]} -> {r.get('status')} {r.get('ms')}ms [{r.get('req')}]")
    if s == "tx":
        if r.get("kind") == "dropped":
            return f"TX !! {r.get('lines')} log lines dropped (queue full)"
        if r.get("kind") == "ioctl":
            what = f"ioctl {r.get('io_type')} {r.get('payload', '')[:40]}"
        elif r.get("msg") == 202:
            what = f"MOTOR ly={r.get('ly')} rx={r.get('rx')}" + (
                f" lx={r.get('lx')} ry={r.get('ry')} btn={r.get('buttons')}"
                if (r.get("lx") or r.get("ry") or r.get("buttons")) else "")
        elif r.get("msg") == 200:
            what = f"COMMAND {r.get('command')}"
        elif r.get("msg") == 229:
            what = f"PARAM {r.get('param')}={r.get('value')}"
        else:
            what = f"msg{r.get('msg')} {r.get('payload', r.get('raw', ''))[:40]}"
        return f"TX   {what}  <{r.get('origin')}>" + (f" [{r.get('req')}]" if r.get("req") else "")
    return f"RX   msg{r.get('msg')} len={r.get('len')} {r.get('payload', '')[:48]}"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--since")
    ap.add_argument("--until")
    ap.add_argument("--last", type=float, help="the last N seconds")
    ap.add_argument("--dir", default=DEF_DIR)
    ap.add_argument("--no-rx", action="store_true")
    ap.add_argument("--no-http", action="store_true")
    ap.add_argument("--raw", action="store_true", help="don't collapse identical consecutive TX frames")
    a = ap.parse_args(argv)
    today = dt.date.today()
    now = time.time()
    t1 = parse_t(a.until, today) if a.until else now
    t0 = parse_t(a.since, today) if a.since else (now - a.last if a.last else now - 300)
    recs = load(a.dir, "tx", t0, t1)
    if not a.no_http:
        recs += load(a.dir, "http", t0, t1)
    if not a.no_rx:
        recs += load(a.dir, "rx", t0, t1)
    recs.sort(key=lambda r: r.get("ts", 0))
    lines = []
    i = 0
    while i < len(recs):
        r = recs[i]
        if r["_s"] != "tx" or a.raw:
            lines.append(f"{fmt_ts(r['ts'])}  {describe(r)}")
            i += 1
            continue
        # collapse a run of identical TX frames; HTTP/RX lines that fall inside the run (same ms,
        # heartbeats) are printed right after the collapsed line, a different TX frame ends the run
        k, j, n, last, between = tx_key(r), i + 1, 1, r, []
        while j < len(recs):
            q = recs[j]
            if q["_s"] == "tx":
                if tx_key(q) != k:
                    break
                n, last = n + 1, q
            else:
                between.append(q)
            j += 1
        tail = f"  ×{n} until {fmt_ts(last['ts'])}" if n > 1 else ""
        lines.append(f"{fmt_ts(r['ts'])}  {describe(r)}{tail}")
        lines.extend(f"{fmt_ts(q['ts'])}  {describe(q)}" for q in between)
        i = j
    print("\n".join(lines) if lines else "(no log lines in that window)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
