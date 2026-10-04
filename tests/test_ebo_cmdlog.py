import importlib
import io
import json
import os
import struct
import sys
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))
sys.path.insert(0, str(ROOT / "tools"))


def fresh_log(tmp, **env):
    os.environ["EBO_LOG_DIR"] = tmp
    for k, v in env.items():
        os.environ[k] = v
    import ebo_cmdlog
    m = importlib.reload(ebo_cmdlog)
    for k in env:
        os.environ.pop(k, None)
    return m


def motor(lx=0.0, ly=0.0, rx=0.0, ry=0.0, buttons=0):
    p = struct.pack("<ffff", lx, -ly, rx, ry) + bytes([buttons])
    return bytes([0xFE, len(p), 0, 0, 0, 202]) + p + b"\x00\x00"


def read(tmp, stream):
    with open(os.path.join(tmp, f"{stream}.jsonl")) as f:
        return [json.loads(l) for l in f]


class CmdLogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="ebolog-unit-")

    def test_tx_decode_and_origin(self):
        cl = fresh_log(self.tmp)
        cl.log_tx_mavlink(motor(rx=1.0))
        with cl.origin("warmup"):
            cl.log_tx_mavlink(motor())
        cl.log_tx_mavlink(bytes([0xFE, 4, 0, 0, 0, 200]) + struct.pack("<H", 40154) + bytes([255, 1]) + b"\0\0")
        cl.flush()
        tx = read(self.tmp, "tx")
        self.assertEqual(tx[0]["msg"], 202)
        self.assertEqual((tx[0]["rx"], tx[0]["ly"]), (1.0, 0.0))
        self.assertTrue(tx[0]["origin"].startswith("thread:"))
        self.assertEqual(tx[1]["origin"], "warmup")
        self.assertEqual(tx[2]["command"], 40154)

    def test_ly_sign_is_public_convention(self):
        cl = fresh_log(self.tmp)
        cl.log_tx_mavlink(motor(ly=0.6))
        cl.flush()
        self.assertEqual(read(self.tmp, "tx")[0]["ly"], 0.6)

    def test_never_blocks_when_writer_is_slow(self):
        cl = fresh_log(self.tmp)
        cl._q = __import__("queue").Queue(maxsize=50)
        orig = cl._write
        cl._write = lambda s, r: (time.sleep(0.05), orig(s, r))
        t0 = time.monotonic()
        for _ in range(2000):
            cl.log_tx_mavlink(motor(rx=0.5))
        self.assertLess(time.monotonic() - t0, 0.5, "logging must not block the send path")
        self.assertGreater(cl._dropped, 0)

    def test_rotation(self):
        cl = fresh_log(self.tmp, EBO_LOG_MAX_MB="0.001", EBO_LOG_KEEP="3")
        for _ in range(400):
            cl.log_tx_mavlink(motor(rx=0.3))
        cl.flush(5)
        names = sorted(os.listdir(self.tmp))
        self.assertIn("tx.jsonl.1", names)
        self.assertNotIn("tx.jsonl.5", names)
        self.assertLessEqual(len([n for n in names if n.startswith("tx.jsonl")]), 4)

    def test_rx_dedupe_and_heartbeat_rate(self):
        cl = fresh_log(self.tmp)
        for _ in range(5):
            cl.log_rx(207, b"\x01\x02")          # unchanged battery -> logged once
        cl.log_rx(207, b"\x01\x03")              # change -> logged
        for _ in range(20):
            cl.log_rx(222, b"\x00")              # heartbeat -> every 10th
        cl.log_rx(163, b"\xaa" * 16)             # unknown/other -> always
        cl.flush()
        ids = [r["msg"] for r in read(self.tmp, "rx")]
        self.assertEqual(ids.count(207), 2)
        self.assertEqual(ids.count(222), 2)
        self.assertEqual(ids.count(163), 1)

    def test_middleware_binds_request_id_and_origin(self):
        cl = fresh_log(self.tmp)
        from starlette.applications import Starlette
        from starlette.responses import JSONResponse
        from starlette.routing import Route
        from starlette.testclient import TestClient

        async def move(request):
            await request.body()                 # FastAPI endpoints with a JSON model read the body
            cl.log_tx_mavlink(motor(rx=0.4))
            return JSONResponse({"ok": True})
        app = Starlette(routes=[Route("/move", move, methods=["POST"])])
        app.add_middleware(cl.CommandLogMiddleware)
        r = TestClient(app).post("/move", json={"rx": 0.4, "duration": 0.2}, auth=("juraj", "x"))
        self.assertEqual(r.status_code, 200)
        cl.flush()
        http, tx = read(self.tmp, "http"), read(self.tmp, "tx")
        self.assertEqual(http[0]["path"], "/move")
        self.assertEqual(http[0]["user"], "juraj")
        self.assertIn('"rx":0.4', http[0]["body"].replace(' ', ''))
        self.assertEqual(tx[0]["req"], http[0]["req"])
        self.assertEqual(tx[0]["origin"], "POST /move")

    def test_ebolog_merges_and_collapses(self):
        cl = fresh_log(self.tmp)
        with cl.origin("warmup"):
            for _ in range(30):
                cl.log_tx_mavlink(motor())
        cl.log_tx_mavlink(motor(rx=1.0))
        cl.log_http({"ts": time.time(), "req": "abc", "client": "10.22.33.5:1", "user": "u",
                     "method": "POST", "path": "/move", "body": "{}", "status": 200, "ms": 3})
        cl.flush()
        import ebolog
        buf = io.StringIO()
        with redirect_stdout(buf):
            ebolog.main(["--dir", self.tmp, "--last", "60"])
        out = buf.getvalue()
        self.assertIn("×30", out)
        self.assertIn("MOTOR ly=0.0 rx=1.0", out)
        self.assertIn("HTTP 10.22.33.5:1", out)


if __name__ == "__main__":
    unittest.main()
