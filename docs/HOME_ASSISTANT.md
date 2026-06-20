# Home Assistant integration

The bridge integrates over the LAN — no add-on is installed on the HA host (the code is
ARM; your HA likely runs on x86). HA consumes two things: **MQTT** (entities) and **RTSP**
(camera).

## 1. MQTT (control + sensors)

Point the bridge at your existing MQTT broker in `.env`:

```
EBO_MQTT_HOST=<ip-of-your-ha-or-broker>
EBO_MQTT_PORT=1883
EBO_MQTT_USER=<broker user>
EBO_MQTT_PASS=<broker pass>
```

In Home Assistant, make sure the **MQTT** integration is configured (Settings → Devices &
Services → Add Integration → MQTT). The bridge publishes Home Assistant **discovery**
messages, so a device named **EBO-SE** appears automatically with:

- **Buttons:** Wake, Sleep, Dock, Stop docking, Forward, Backward, Left, Right
- **Switches:** Eye lights, Night vision, Collision avoidance, Fall protection, Patrol
- **Sensors:** Battery (%), Charging, Connected
- **Diagnostics:** Name, Model, Serial, FSN, MCU version, Camera firmware, TUTK UID,
  Wi-Fi SSID, IP, MAC, RTSP URL

No robot credentials are entered in HA — the bridge holds them.

## 2. Camera (video)

HA does not create RTSP cameras via MQTT discovery, so add it once:

**Settings → Devices & Services → Add Integration → Generic Camera**
- Stream Source (RTSP): `rtsp://<stream_user>:<stream_pass>@<pi-ip>:8554/ebo`
- RTSP transport: TCP

This creates `camera.ebo_se`. HA's built-in **go2rtc** handles RTSP → WebRTC for the
dashboard. The stream is HEVC; most modern browsers/apps decode it in hardware.

## 3. Dashboard

Add a **Picture/Camera** card for `camera.ebo_se`, and the EBO-SE buttons/switches for
control. For low-latency manual driving (d-pad), use the bridge's own panel.

> **Embedding the panel in an HTTPS dashboard.** If Home Assistant is served over HTTPS, a
> **Webpage** card pointing at `http://<pi-ip>:8000` is blocked (mixed content). Expose the
> panel over HTTPS instead — e.g. a **Cloudflare Tunnel** mapping `panel.example.com` →
> `http://localhost:8000` (free, auto-renewing cert, no open ports), then use that HTTPS URL in
> the Webpage card. Keep `EBO_BIND=127.0.0.1` so the panel is reachable only through the tunnel.
>
> **WebRTC video note:** the panel's live video uses WebRTC; the media flows peer-to-peer
> (browser ↔ Pi), not through the tunnel. It plays when your browser is on the same LAN as the
> Pi. For video when away from home you'd need a TURN server or WebRTC-over-TCP — the buttons,
> d-pad and the HA camera (RTSP) are unaffected.

### Sleep fallback image

When the robot sleeps, the camera stops and HA shows the camera as unavailable. The bridge
publishes an **Awake** binary sensor (`binary_sensor.*_awake`, ON while streaming) and serves a
ready-made *"Robot in sleep mode"* graphic at `/sleep.svg` (public, no auth). Use a conditional
stack to swap the camera for the graphic while asleep:

```yaml
type: vertical-stack
cards:
  - type: conditional
    conditions: [{ entity: binary_sensor.ebo_se_awake, state: "on" }]
    card:
      type: picture-entity
      entity: camera.ebo_se
      camera_view: live
  - type: conditional
    conditions: [{ entity: binary_sensor.ebo_se_awake, state: "off" }]
    card:
      type: picture
      image: https://<your-panel-host>/sleep.svg     # or /local/ebo_sleep.svg (see below)
```

Adjust the entity ids to match yours. If you prefer not to depend on the panel host, download
`app/sleep.svg` into HA's `config/www/` folder and reference it as `/local/ebo_sleep.svg`.

## Automations (examples)

- *At 22:00 → press `button.ebo_dock`* (send the robot home).
- *On motion detected → press `button.ebo_wake` + turn on `switch.ebo_patrol`.*
- *When `sensor.ebo_battery` < 20% and not charging → notify.*

## Troubleshooting

- **No entities:** check the bridge log (`docker logs ebo-bridge`) shows `MQTT -> <host>`
  and `connected`; verify broker host/credentials.
- **Camera black / not playing:** verify the RTSP URL + credentials; if your browser can't
  decode HEVC, enable an H.264 transcode (the Pi 4 has `h264_v4l2m2m`) — see the notes in
  `app/ebo_server.py`.
- **Controls do nothing:** make sure the ROLA phone app is closed (single session).
