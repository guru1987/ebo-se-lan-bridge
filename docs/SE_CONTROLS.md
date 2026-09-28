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
- `GET /security/tasks/1` reads fixed-point tasks; type `2` reads cruise tasks.
- `PUT /security/tasks` creates or updates an app-compatible schedule.

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

## Home Assistant MQTT

Discovery publishes state-backed switches plus number entities for speaker
volume, call volume, drive speed, security sensitivity, and notification
interval. Switch and number state comes from robot echoes, not optimistic UI
updates.
