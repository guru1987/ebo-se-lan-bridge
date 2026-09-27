#!/usr/bin/env python3
"""Send a short, low-volume tone through the authenticated talkback WebSocket."""
import asyncio
import base64
import json
import math
import os
import struct

import websockets


RATE = 8000
SAMPLES_PER_FRAME = 640  # 80 ms, matching ROLA's nominal capture chunk
FRAME_COUNT = 20
AMPLITUDE = 1200
FREQUENCY = 1000


def pcm_frame() -> bytes:
    samples = (
        int(AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * index / RATE))
        for index in range(SAMPLES_PER_FRAME)
    )
    return b"".join(struct.pack("<h", sample) for sample in samples)


async def main() -> None:
    user = os.environ.get("EBO_WEB_USER", "")
    password = os.environ.get("EBO_WEB_PASS", "")
    if not user:
        raise SystemExit("EBO_WEB_USER must be set for the smoke test")
    token = base64.b64encode(f"{user}:{password}".encode()).decode()
    uri = os.environ.get("EBO_TALK_URL", "ws://127.0.0.1:8000/ws/talk")
    async with websockets.connect(
        uri, additional_headers={"Authorization": "Basic " + token}
    ) as websocket:
        response = json.loads(await websocket.recv())
        if response.get("type") != "ready":
            raise RuntimeError("talkback did not become ready")
        frame = pcm_frame()
        for _ in range(FRAME_COUNT):
            await websocket.send(frame)
            await asyncio.sleep(SAMPLES_PER_FRAME / RATE)
    print("talkback websocket smoke: ok")


if __name__ == "__main__":
    asyncio.run(main())
