# EBO-SE Bridge

A **phone-free** bridge for the **Enabot EBO SE** robot. It runs the official TUTK/Kalay
libraries on an ARM Linux host and exposes the robot to **Home Assistant** as a camera and a
set of native entities — **no phone, no ROLA app, no cloud account online** required for use.

It connects directly to the robot over the LAN (Kalay P2P + DTLS-PSK), receives the live
**H.265 1080p30 video** with **G.711 microphone audio**, provides **push-to-talk audio to
the robot speaker**, and lets you **drive it and
control it** (analog joystick + d-pad, dock, wake/sleep, eye lights, night vision,
collision/fall protection, patrol), with **battery** and **diagnostic** sensors.

> You provide your own robot credentials and the TUTK libraries (extracted from your own
> device). This repo contains only the integration code — see [docs/SETUP.md](docs/SETUP.md).

## How it works

Home Assistant can run on a separate machine (often x86); the bridge runs on an ARM host
with a native bridge, bionic runtime and owned TUTK libraries of one matching ABI. The
original deployment is ARM32; ARM64 is supported for hardware such as the ODROID-C2.

```
 Robot ◄──Kalay P2P / DTLS──► ebo_bridge (native, bionic)
                                 │ H.265/G.711             ▲ control + speaker PCM (fd3)
                                 ▼                         │
                            ffmpeg -c copy ──► mediamtx (RTSP/WebRTC/HLS)
                                 │                        │
   Home Assistant ◄── RTSP camera ──┘     MQTT entities ──┴── ebo_server.py (supervisor)
   (on your mini-PC)  ◄── MQTT discovery ──────────────────────────┘
```

- **The bridge host holds all the secrets.** Home Assistant only talks to MQTT (entities) and pulls
  the RTSP stream. It never authenticates with the robot.
- **Video is passthrough** (`ffmpeg -c:v copy`): the bridge host does *no* decoding/transcoding
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
| `vendor/` | **you provide**: TUTK `.so`, bionic runtime, optional `ioctl9930.bin` (gitignored) |

## Quick start (ARM host)

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

The web panel (video, manual driving and hold-to-talk) binds to `127.0.0.1:8000` by default and is meant to be
exposed over **HTTPS** via a reverse proxy or **Cloudflare Tunnel** (so it can be embedded in
an HTTPS Home Assistant dashboard without mixed-content errors). To reach it directly on the
LAN over plain HTTP instead, set `EBO_BIND=0.0.0.0` and open `http://<pi-ip>:8000`.

Browser microphone capture requires a secure context: use HTTPS, or `localhost` while
developing. Talkback accepts one browser under the panel's configured access policy at a
time and always sends the native
speaker STOP command when the button is released or the WebSocket disconnects. See
[docs/TALKBACK.md](docs/TALKBACK.md) for the tested wire format and validation notes.
For an x86-64, non-Docker deployment see [docs/HOST_QEMU.md](docs/HOST_QEMU.md).

## Security

RTSP, WebRTC, the web panel and MQTT are all authenticated (credentials in `.env`).
Keep the bridge host on a trusted network; the stream/control are only as private as those credentials.
The local ffmpeg publisher is admitted anonymously only from loopback, keeping reader
credentials out of process arguments.

## Status

Video, robot microphone audio, control, battery and diagnostics have been tested on the
original ARM32 Raspberry Pi path. The new speaker path was live-tested with the owned ARM64
ROLA libraries on a rooted Pixel 3: channel 2 became ready, audio frames were accepted, STOP
completed, and the process exited cleanly. The ARM64 ODROID-C2 container deployment remains
to be validated on that host.

## Credits

- **Original project and core bridge:** [lilium360](https://github.com/lilium360)
- **Two-way audio research and implementation:** GPT-5.6-Sol by OpenAI
- Opus 5.5 by Anthropic
- [Same Sun Foundation](https://www.samesun.foundation)
- From Porch in Nuremberg, by Brothers: Grandpa, Phil, Ghost, Fourth

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
