# EBO-SE Bridge

A **phone-free** bridge for the **Enabot EBO SE** robot. It runs the official TUTK/Kalay
libraries on a Raspberry Pi and exposes the robot to **Home Assistant** as a camera and a
set of native entities — **no phone, no ROLA app, no cloud account online** required for use.

It connects directly to the robot over the LAN (Kalay P2P + DTLS-PSK), receives the live
**H.265 1080p30 video** with **G.711 audio (listen-only)**, and lets you **drive it and
control it** (analog joystick + d-pad, dock, wake/sleep, eye lights, night vision,
collision/fall protection, patrol), with **battery** and **diagnostic** sensors.

> You provide your own robot credentials and the TUTK libraries (extracted from your own
> device). This repo contains only the integration code — see [docs/SETUP.md](docs/SETUP.md).

## How it works

Home Assistant runs on a separate machine (often x86); the bridge runs on a Raspberry Pi
(ARM), because the TUTK libraries are 32-bit ARM/Android. HA integrates over the LAN.

```
 Robot ──Kalay P2P / DTLS──► ebo_bridge (native, bionic)
                                 │ H.265 video            ▲ control (fd3, RDT/MAVLink)
                                 ▼                        │
                            ffmpeg -c copy ──► mediamtx (RTSP/WebRTC/HLS)
                                 │                        │
   Home Assistant ◄── RTSP camera ──┘     MQTT entities ──┴── ebo_server.py (supervisor)
   (on your mini-PC)  ◄── MQTT discovery ──────────────────────────┘
```

- **The Pi holds all the secrets.** Home Assistant only talks to MQTT (entities) and pulls
  the RTSP stream. It never authenticates with the robot.
- **Video is passthrough** (`ffmpeg -c:v copy`): the Pi does *no* decoding/transcoding
  (~5% CPU). The viewer (browser / HA) decodes HEVC in hardware.
- **Battery/status** comes from the robot over the reliable channel (`RDT_Read`).

## Components

| File | Role |
|------|------|
| `app/ebo_bridge.c` / `app/ebo_bridge` | native client: connects, video→stdout, control←fd3 |
| `app/ebo_server.py` | supervisor: spawns mediamtx+ffmpeg+bridge, REST API, web panel |
| `app/ebo_mqtt.py` | Home Assistant MQTT Discovery (entities) |
| `app/ebo.html` | web panel (live video + d-pad) |
| `app/mediamtx.template.yml` | RTSP/WebRTC/HLS server config (auth injected at start) |
| `app/run.sh` | container entrypoint |
| `Dockerfile`, `docker-compose.yml` | packaging |
| `vendor/` | **you provide**: TUTK `.so`, bionic runtime, `ioctl9930.bin` (gitignored) |

## Quick start (Raspberry Pi)

```bash
git clone <this-repo> && cd ebo-se-bridge

# 1) Populate ./vendor (TUTK libs + bionic runtime + ioctl9930.bin)
#    See docs/SETUP.md
# 2) Configure
cp .env.example .env && nano .env          # robot creds + stream/web/MQTT auth
# 3) Build & run
docker compose up -d --build
```

Then in Home Assistant: **Settings → Devices & Services → Add Integration → MQTT**
(point it at your broker) — the **EBO-SE** device and all entities appear automatically.
Add the camera with **Generic Camera** → `rtsp://<stream_user>:<stream_pass>@<pi-ip>:8554/ebo`.
Full guide: [docs/HOME_ASSISTANT.md](docs/HOME_ASSISTANT.md).

The web panel (manual driving) binds to `127.0.0.1:8000` by default and is meant to be
exposed over **HTTPS** via a reverse proxy or **Cloudflare Tunnel** (so it can be embedded in
an HTTPS Home Assistant dashboard without mixed-content errors). To reach it directly on the
LAN over plain HTTP instead, set `EBO_BIND=0.0.0.0` and open `http://<pi-ip>:8000`.

## Security

RTSP, WebRTC, the web panel and MQTT are all authenticated (credentials in `.env`).
Keep the Pi on a trusted network; the stream/control are only as private as those credentials.

## Status

Working and tested on a Raspberry Pi 4: video (RTSP, fluid), full control, battery and
diagnostics, all as native Home Assistant entities.

## Legal & disclaimer

**Not affiliated.** This is an independent, community project. It is **not** affiliated with,
authorized, sponsored, or endorsed by Enabot or ThroughTek. *Enabot*, *EBO*, *ROLA*, *TUTK* and
*Kalay* are trademarks of their respective owners; they are used here only nominatively, to
describe interoperability with those products.

**No proprietary components are redistributed.** This repository contains **only original code**.
The TUTK/Kalay SDK and the Enabot/ROLA application are proprietary and are **not** included,
copied, or derived here — no SDK libraries, no Android runtime, no app code, no keys. You obtain
those components yourself, **from a device and software you own and are licensed to use** (see
[docs/SETUP.md](docs/SETUP.md)).

**Interoperability & personal use.** This project exists to interoperate with **hardware you own**,
for personal use and research. You are solely responsible for complying with your local laws and
with the terms of service / EULA of the Enabot app and the ThroughTek SDK when using your own
device and credentials. **Use at your own risk.**

**License.** The original code in this repository is released under the **MIT License** (see
[LICENSE](LICENSE)). The MIT license covers *this project's code only* — it grants **no rights** to
any third-party component (TUTK SDK, Android runtime, etc.), which remain under their own licenses.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. See [LICENSE](LICENSE).
