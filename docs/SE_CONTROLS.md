# EBO SE settings, security tasks, and telemetry

This bridge implements the private MAVLink-v1 messages used by the ROLA app.
It does not require Frida or changes to the Android app. Parameter reads and
writes are stateful: a write succeeds only after the robot echoes the value.

## HTTP API

The API uses the same HTTP Basic authentication as the web panel.

- `GET /parameters` returns cached values, metadata, and the supported allowlist.
- `GET /parameter/{group}/{key}` asks the robot for one current value.
- `PUT /parameter/{group}/{key}` writes `{"value": 50}` and waits for its echo.
- `GET /telemetry` returns the latest decoded status messages.
- `GET /security/tasks/1` reads fixed-point tasks; type `2` reads cruise and type `3` reads do-not-disturb tasks.
- `PUT /security/tasks` creates or updates an app-compatible schedule.
- `POST /action/autoreturn_off` installs an all-day do-not-disturb task, suppressing automatic battery return; `autoreturn_on` disables it and restores the prior in-memory schedule when available.
- `GET /auto-return` reports the derived auto-return state and the underlying type-3 tasks.
- `GET /day-night` returns the configured camera policy; actions `night_auto`, `day`/`night_off`, and `night_on` select auto (`0`), forced colour (`1`), and forced IR/mono (`2`).
- `POST /video/stop` and `/video/start` stop/start video IOCTRL only. The separate robot-audio and talkback channels remain connected. `/video/start` waits for a new publishable frame (or returns 504 after `EBO_VIDEO_READY_TIMEOUT`, default 12 seconds); `GET /video/ready` provides non-mutating readiness state.
- The existing `rtsp://BRIDGE:8554/ebo` path remains unchanged. Always-on microphone audio is also published independently at `rtsp://BRIDGE:8554/ebo_audio`, so audio readers do not stall when video is stopped.
- `POST /photo` asks the robot IPC to save its original 1920x1080 frame under `/DICM`.
- `GET /media` lists SD-card photos/videos; `GET /media/{name}` downloads an unambiguous basename.

Supported settings include speaker volume, call volume, drive speed, eye
lights, night vision, obstacle/fall protection, fixed-point and cruise patrol,
security notifications, sensitivity, notification interval, and the four
normalized security-area coordinates (0–100).

ROLA encodes security sensitivity as `0`, `2`, or `4`. Notification intervals
are seconds (`0`, `60`, `180`, `300`, `600`, or a custom whole-minute value up
to `3600`); `-1` is the app's special disabled/unlimited choice.

Example parameter write:

```sh
curl -u "$EBO_WEB_USER:$EBO_WEB_PASS" \
  -X PUT -H 'content-type: application/json' \
  -d '{"value":75}' http://ROBOT_BRIDGE:8000/parameter/audio/volume
```

Example three-minute cruise schedule (all seven weekdays):

```sh
curl -u "$EBO_WEB_USER:$EBO_WEB_PASS" \
  -X PUT -H 'content-type: application/json' \
  -d '{"task_id":0,"task_type":2,"enabled":true,"weekdays":127,
       "start_hour":12,"start_minute":0,"end_hour":12,"end_minute":3}' \
  http://ROBOT_BRIDGE:8000/security/tasks
```

**Safety:** an enabled cruise schedule may move the robot autonomously at the
configured time. Keep it on the floor with stairs and drop-offs inaccessible.
The bridge does not activate a cruise immediately when saving the schedule.

## Telemetry

The streaming decoder tolerates RDT fragmentation, multiple MAVLink frames per
chunk, noise, and corrupt checksums. Currently decoded fields include detailed
battery voltages/current/charge state, attitude, acceleration, gyroscope,
rangefinder (when available), speed level, wheel speed/circles, motor
temperatures/overtemperature flags, camera state, and MCU mode.

EBO SE hardware varies. Fields such as rangefinder simply remain absent when a
robot does not report them; this bridge does not assume an SE2 laser exists.

The APK exposes the configured day/night policy but no confirmed sensor message
for the IR-cut mechanism's instantaneous state while automatic mode is active.
Use forced day mode when colour output is required.

## SD-card protocol

Photo capture is TUTK IOCTRL `0x9901` with an eight-byte zero payload. Media
access first sends MAVLink `FILE_SERVER_CMD_OPEN` (message 40) over RDT. The
robot replies with a temporary LAN address, port, and 32-byte password in
message 42; the bridge then logs in and uses MAVLink `FILE_TRANSFER_PROTOCOL`
(message 218) on that TCP socket. No Android filesystem access is involved.

Downloads are capped at 512 MiB by default; set `EBO_MEDIA_MAX_BYTES` to change
that limit. Temporary downloads are deleted after the HTTP response completes.

## Auto-return limitation

ROLA does not expose a standalone `auto_return` parameter. Its documented
suppression mechanism is do-not-disturb task type `3`. Therefore the switch is
derived: "off" enables a seven-day 00:00–23:59 DND task, and "on" disables it.
This may also suppress other firmware activity covered by do-not-disturb.

## Home Assistant MQTT

Discovery publishes state-backed switches plus number entities for speaker
volume, call volume, drive speed, security sensitivity, and notification
interval. Switch and number state comes from robot echoes, not optimistic UI
updates.
