"""Always-on command log for the bridge: every frame to the robot (TX), every message from it (RX)
and every HTTP request, as JSONL in EBO_LOG_DIR (default ~/ebo_bridge/runtime/logs/{tx,rx,http}.jsonl).

Never blocks the send path: callers only put a dict on a bounded queue; one writer thread serialises,
appends and size-rotates (EBO_LOG_MAX_MB per file, EBO_LOG_KEEP old files). If the queue is full the
line is dropped and counted (reported as a "dropped" line once the queue drains).

Origin of a TX frame = the code path that sent it: an explicit tag (origin("warmup") / origin("watchdog")),
else the HTTP request that caused it ("POST /move"), else the sending thread's name ("thread:dockwatch").
"""
import contextlib
import contextvars
import json
import os
import queue
import struct
import threading
import time
import uuid

LOG_DIR = os.path.expanduser(os.environ.get("EBO_LOG_DIR", "~/ebo_bridge/runtime/logs"))
MAX_BYTES = int(float(os.environ.get("EBO_LOG_MAX_MB", "50")) * 1024 * 1024)
KEEP = int(os.environ.get("EBO_LOG_KEEP", "10"))
ENABLED = os.environ.get("EBO_CMDLOG", "1") != "0"
HEARTBEAT_EVERY = 10          # RX msg 222 (heartbeat): log every 10th
CHANGE_ONLY = {203, 207}      # RX status / battery: log only when the payload changes

_request_id = contextvars.ContextVar("ebo_request_id", default=None)
_request_tag = contextvars.ContextVar("ebo_request_tag", default=None)
_origin = contextvars.ContextVar("ebo_origin", default=None)

_q = queue.Queue(maxsize=20000)
_dropped = 0
_files = {}
_writer = None
_wlock = threading.Lock()
_rx_last = {}
_rx_count = {}


def _now():
    return round(time.time(), 3)


@contextlib.contextmanager
def origin(tag):
    """Tag every TX frame sent inside this block (same thread) with `tag`."""
    tok = _origin.set(tag)
    try:
        yield
    finally:
        _origin.reset(tok)


def current_origin():
    return _origin.get() or _request_tag.get() or ("thread:" + threading.current_thread().name)


def _put(stream, rec):
    global _dropped
    if not ENABLED:
        return
    _ensure_writer()
    try:
        _q.put_nowait((stream, rec))
    except queue.Full:
        _dropped += 1


# ---------------- decoding ----------------
def decode_mavlink(frame):
    """Decode a MAVLink v1 frame built by the bridge (0xfe, len, seq, sys, comp, msgid, payload, crc)."""
    try:
        if len(frame) < 8 or frame[0] != 0xFE:
            return {"raw": bytes(frame).hex()}
        n, mid = frame[1], frame[5]
        p = bytes(frame[6:6 + n])
        out = {"msg": mid}
        if mid == 202 and len(p) >= 17:
            lx, ly_native, rx, ry = struct.unpack("<ffff", p[:16])
            out.update(lx=round(lx, 3), ly=round(-ly_native, 3), rx=round(rx, 3), ry=round(ry, 3), buttons=p[16])
        elif mid == 200 and len(p) >= 2:
            out.update(command=struct.unpack("<H", p[:2])[0])
        elif mid == 229 and len(p) >= 38:
            out.update(value=round(struct.unpack("<f", p[:4])[0], 4),
                       param=p[6:38].rstrip(b"\x00").decode(errors="replace"), ptype=p[38] if len(p) > 38 else None)
        else:
            out.update(payload=p.hex())
        return out
    except Exception as e:   # never let logging break a send
        return {"raw": bytes(frame).hex(), "decode_error": str(e)}


def log_tx_mavlink(frame):
    _put("tx", {"ts": _now(), "kind": "mavlink", **decode_mavlink(frame),
                "origin": current_origin(), "req": _request_id.get()})


def log_tx_ioctl(io_type, data):
    _put("tx", {"ts": _now(), "kind": "ioctl", "io_type": io_type, "payload": bytes(data).hex()[:200],
                "origin": current_origin(), "req": _request_id.get()})


def log_rx(msgid, payload):
    payload = bytes(payload)
    if msgid in CHANGE_ONLY:
        if _rx_last.get(msgid) == payload:
            return
        _rx_last[msgid] = payload
    elif msgid == 222:
        c = _rx_count[msgid] = _rx_count.get(msgid, 0) + 1
        if c % HEARTBEAT_EVERY != 1:
            return
    rec = {"ts": _now(), "msg": msgid, "len": len(payload), "payload": payload.hex()[:400]}
    if msgid == 207 and len(payload) >= 4:
        rec["decoded"] = "battery"
    _put("rx", rec)


def log_http(rec):
    _put("http", rec)


# ---------------- ASGI middleware ----------------
class CommandLogMiddleware:
    """Logs every HTTP request (client, user, method, path, short body, status, duration, request id)
    and binds the request id + "METHOD /path" as the origin of TX frames sent while handling it."""

    def __init__(self, app, body_max=300):
        self.app = app
        self.body_max = body_max

    async def __call__(self, scope, receive, send):
        if scope.get("type") != "http":
            return await self.app(scope, receive, send)
        rid = uuid.uuid4().hex[:10]
        method, path = scope.get("method", ""), scope.get("path", "")
        t_rid = _request_id.set(rid)
        t_tag = _request_tag.set(f"{method} {path}")
        t0 = time.monotonic()
        body = bytearray()
        status = {"code": None}

        async def recv():
            msg = await receive()
            if msg.get("type") == "http.request" and len(body) < self.body_max:
                body.extend(msg.get("body", b"")[: self.body_max - len(body)])
            return msg

        async def snd(msg):
            if msg.get("type") == "http.response.start":
                status["code"] = msg.get("status")
            await send(msg)

        try:
            await self.app(scope, recv, snd)
        finally:
            client = scope.get("client") or ("?", 0)
            user = None
            for k, v in scope.get("headers") or []:
                if k == b"authorization" and v[:6].lower() == b"basic ":
                    try:
                        import base64
                        user = base64.b64decode(v[6:]).decode(errors="replace").split(":", 1)[0]
                    except Exception:
                        user = "?"
            q = scope.get("query_string", b"").decode(errors="replace")
            log_http({"ts": _now(), "req": rid, "client": f"{client[0]}:{client[1]}", "user": user,
                      "method": method, "path": path + (("?" + q) if q else ""),
                      "body": body.decode(errors="replace"), "status": status["code"],
                      "ms": round((time.monotonic() - t0) * 1000, 1)})
            _request_tag.reset(t_tag)
            _request_id.reset(t_rid)


# ---------------- writer ----------------
def _open(stream):
    f = _files.get(stream)
    if f is None:
        os.makedirs(LOG_DIR, exist_ok=True)
        f = _files[stream] = open(os.path.join(LOG_DIR, f"{stream}.jsonl"), "a", buffering=1)
    return f


def _rotate(stream):
    f = _files.pop(stream, None)
    if f:
        f.close()
    base = os.path.join(LOG_DIR, f"{stream}.jsonl")
    for i in range(KEEP - 1, 0, -1):
        src = f"{base}.{i}"
        if os.path.exists(src):
            os.replace(src, f"{base}.{i + 1}")
    if os.path.exists(base):
        os.replace(base, f"{base}.1")
    old = f"{base}.{KEEP + 1}"
    if os.path.exists(old):
        os.remove(old)


def _write(stream, rec):
    f = _open(stream)
    f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    if f.tell() >= MAX_BYTES:
        _rotate(stream)


def _run():
    global _dropped
    while True:
        stream, rec = _q.get()
        try:
            _write(stream, rec)
            if _dropped and _q.empty():
                n, _dropped = _dropped, 0
                _write("tx", {"ts": _now(), "kind": "dropped", "lines": n})
        except Exception:
            pass


def _ensure_writer():
    global _writer
    if _writer is None:
        with _wlock:
            if _writer is None:
                _writer = threading.Thread(target=_run, name="cmdlog-writer", daemon=True)
                _writer.start()


def flush(timeout=2.0):
    """Wait until the queue is written (tests / shutdown)."""
    t = time.monotonic() + timeout
    while not _q.empty() and time.monotonic() < t:
        time.sleep(0.01)
    time.sleep(0.02)
    for f in list(_files.values()):
        try:
            f.flush()
        except Exception:
            pass
