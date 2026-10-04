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
- `PUT /video/quality` with `{"level":"auto|high|medium|low"}` selects ROLA's persisted live-view profile. `GET /video/quality` reports the last bridge request and known targets; the firmware does not echo its persisted selection.
- The existing `rtsp://BRIDGE:8554/ebo` path remains unchanged. Always-on microphone audio is also published independently at `rtsp://BRIDGE:8554/ebo_audio`, so audio readers do not stall when video is stopped.
- `POST /photo` asks the robot IPC to save its original 1920x1080 frame under `/DICM`.
- `GET /media` lists SD-card photos/videos; `GET /media/{name}` downloads an unambiguous basename.

## Robot Skills (autonomous movement)

`POST /action/big_circle` (aliases `/action/rotate` and `/action/donut`),
`/action/straight_line` (aliases `/action/rush_forward` and `/action/rocket`),
and `/action/swing` (alias `/action/shake`) invoke ROLA's three direct motion
buttons (commands 40115,
40116, and 40129). `/action/spin`, `/action/race`, `/action/excited`,
`/action/relaxed`, and `/action/little_monster` invoke the five Robot Skills
(commands 40164..40168). All use MAVLink message 200 (`COMMAND_MINI`). These
actions can move or lift the robot abruptly; test on a blocked, level floor
with clearance, direct supervision, and `/stop` ready. The firmware, not the
bridge, controls each program's speed and duration.

Trick gate: `POST /action/<trick>` is refused (`{"ok": false, "refused": true, "reason"}`,
nothing sent) while the robot is charging/docked or its charge state is unknown, and,
when the depth guard is on, unless one fresh depth reading shows `stop_free_m` of room
ahead (a start-only check; the guard cannot stop a running firmware routine). `?force=1`
skips both checks. Every trick call is logged as `[trick] SENT|REFUSED|FAILED {json}`.

`POST /action/undock` drives straight forward off the dock: `EBO_UNDOCK_LY` (0.6) for
`EBO_UNDOCK_S` (2.0 s, about 0.28 m; shorter or lower pulses stay in the robot's dead zone)
through the same forward depth guard as `/move` (refused, or stopped mid-way, when free ahead
< `stop_free_m`), then stops and polls the charge state for up to `EBO_UNDOCK_CHARGE_POLL_S`
(2 s). It answers `{ok, moved_s, guard | stopped_by_guard + reason, charge_before,
charge_after, left_dock}` and is logged as `[undock] {json}`.

Forward moves (`/move`, `/drive`, undock, trick gate) with no fresh depth reading (the depth
decoder idle or the video paused by ebo-eye) first wake the stream: ebo-eye
`POST EBO_GUARD_VIDEO_KEEP_URL` (`/video_keep`), then the bridge's own video start if it is
still not publishing; the guard's `/depth/live` polls keep the depth decoder running. They then
wait until the stream is up plus `EBO_GUARD_WAKE_WAIT_S` (3 s) for a fresh reading (at most
`EBO_GUARD_WAKE_MAX_S`, 10 s, after the wake) and still refuse without one. After a wake only a
fresh reading WITH a floor scale (`free_ahead_m` not null) decides: the first readings after a
wake often have no floor fit yet, so fresh unscaled readings extend the wait to
`EBO_GUARD_SCALE_WAIT_S` (8 s) after the wake (same cap). Logged as `[guard]`.

Dock watch: `POST /action/dock` (and MQTT dock) starts the vendor's firmware dock routine (search
turns, a weaving forward approach camera-first, a 180 degree turn, reversing in), which the forward
guard never sees. A `DockWatch` thread then polls the depth (`EBO_DOCKWATCH_URL`, default the
guard's `/depth/live`) every `EBO_DOCKWATCH_POLL_S` (0.1 s) until charge == 1 or
`EBO_DOCKWATCH_TIMEOUT_S` (120 s) and sends motor-zero stop frames when the free distance in the
depth service's wide corridor (`free_ahead_wide_m`, robot half width + `wide_corridor_margin_m`
per side; falls back to `free_ahead_m`) is below `EBO_DOCKWATCH_STOP_M` (0.25 m) and has shrunk
over the last `EBO_DOCKWATCH_SHRINK_N` (3) fresh readings (no step growing, none larger than
`EBO_DOCKWATCH_MAX_STEP_M` 0.15 m, at least `EBO_DOCKWATCH_MIN_SHRINK_M` 0.02 m in total).
Reversing (growing) and turning (jumping/holding) do not trigger; no reading never stops it. After a
stop it re-sends the stop for `EBO_DOCKWATCH_CONFIRM_S` (3 s) if the distance keeps closing
(`still_closing_after_stop`: the firmware ignored the stop). Whether motor-zero frames interrupt the
firmware routine is untested. State: `GET /dockwatch`, `/info` `dockwatch`; a stop is attached once
to the next `/action` or `/move` result as `dockwatch_notice` ("docking stopped: obstacle 0.20 m
ahead"). Logged as `[dockwatch]`. `EBO_DOCKWATCH=0` disables it.
Dock pre-wake: a paused stream needs ~5-7 s to its first depth reading, while the routine can
dock in under 7 s (first live run: 3 readings, 63 blind polls). So before CMD_DOCK the guard wakes
the stream and waits up to `EBO_DOCK_PREWAKE_S` (8 s, 0 = off) for a fresh reading with a floor
scale, then docks regardless (it never refuses). The result is in the dock response (`prewake`:
waited_s, woke, reading, scaled, free_ahead_m) and in the watch state; the watch also counts
`unscaled` readings (no floor scale even after the depth service's 10 s last-scale fallback).

`GET /actions` lists every accepted action name; tricks carry `"moves": true` and their
command number.

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

## Live-view quality

ROLA sends vendor IOCTRL `0x990a` on the tested EBO SE. Its 12-byte payload is
three little-endian unsigned integers: `(level, bitrate_kbps, 1)`. The older
`0x9906` variant can be selected with `EBO_VIDEO_QUALITY_IOCTL=0x9906`.

| Level | Wire values | Live resolution | Target kb/s | Observed kb/s |
| --- | --- | ---: | ---: | ---: |
| Auto | `(0, 0, 1)` | 848x480 in this run | adaptive | 262 |
| High | `(1, 4096, 1)` | 1920x1080 | 4096 | 2068 |
| Medium | `(2, 1500, 1)` | 848x480 | 1500 | 784 |
| Low | `(3, 768, 1)` | 848x480 | 768 | 364 |

Observed rates are 15-second video-only HEVC payload averages while the robot
faced a mostly static scene. They are below the configured ceilings because the
encoder is variable-rate. Auto is adaptive and may choose differently with
other network or scene conditions.

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

### Dock watch recovery (2026-10-02)
After a dock-watch STOP (obstacle closing in the wide corridor) the bridge tries to help the firmware
routine instead of just stopping: it sidesteps AWAY from where the nearest obstacle is - the lateral
offset of the closest wide-corridor point (`/depth/live` `"free_ahead_wide_lateral_m"`, + = right) when
|offset| >= `EBO_DOCKWATCH_SIDE_LAT_MIN_M` (0.03), else the nearest sector per side over the full visible
range (`"sectors"`, deg < 0 = left, +/-42 deg) differing by >= `EBO_DOCKWATCH_SIDE_MIN_DIFF_M` (0.15),
else the mean per side; still unclear -> no movement, report. It turns 90 deg toward the free side
(`EBO_DOCKWATCH_TURN_RX` 0.6 for `EBO_DOCKWATCH_TURN_S` 1.0 s), drives `EBO_DOCKWATCH_SIDESTEP_M`
(0.20 m at `EBO_DOCKWATCH_SIDESTEP_LY` 0.6 / `EBO_DOCKWATCH_SPEED_MPS` 0.14) through the normal forward
depth guard (refused -> turns 180 deg and tries the other side once; refused again -> "recovery_aborted"),
turns back toward the original heading and relaunches
the dock (pre-wake + a new watch). At most `EBO_DOCKWATCH_RECOVER_MAX` (2) retries, then "failed".
`EBO_DOCKWATCH_RECOVER=0` = stop only (old behaviour).
Pre-dock check: if the pre-wake reading's wide free distance is already < `EBO_DOCKWATCH_STOP_M`, the dock is not sent; the same sidestep runs first (counts as attempt 1) and then relaunches the dock; side unclear -> no move, "blocked before docking, side unclear". Skipped when docked (charge 1) or with recovery off. Events in `GET /dockwatch` (`kind`
recovering / recovery_aborted / failed, `side`, `side_info`, `steps`, `attempt`); messages for the
brother accumulate in the one-time `dockwatch_notice`, e.g. "docking blocked: obstacle 0.20 m on the
left - sidestepped right, retrying (1/2)" / "docking failed after 2 attempts: obstacle 0.24 m ahead".
Needs ebo-depth with `"sectors"` in its live reading; without sectors the side is ambiguous -> no movement.

## Motor frame timing and idle warm-up (2026-10-03)

Measured with an overhead 480 fps camera (porch repo `tools/turncal`, runs 1–6):

- The robot always turns at ~350–470°/s; `rx` sets how long, not how fast. One command turns at most ~95–110°.
- From an **idle link** a move starts 0–0.25 s late (onset sd 85 ms; 3 of 16 short pulses did not move at all).
  After **0.6 s of zero-motion frames** the onset sd is ~25 ms and every pulse moves (run5).
- Frames used to go out every 50 ms and a move was floored to whole 50 ms steps (0.08 s → 0.058 s).

Bridge behaviour now:

| Env | Default | Meaning |
|---|---|---|
| `EBO_MOTOR_FRAME_S` | `0.02` | motor frame period; a move lasts the requested duration rounded to the nearest frame |
| `EBO_WARMUP_S` | `0.6` | zero-motion frames sent before a move when the link was idle (`0` = off) |
| `EBO_WARMUP_IDLE_S` | `1.0` | idle time after which the next move warms up first |

Applies to `/move`, `/drive` (the first call from idle warms up; concurrent calls get `{"warming": true}`),
`undock` and the dock-watch recovery (all go through `do_move`). Forward moves are decided by the depth
guard **before** the warm-up; a refusal never warms up, and a guard verdict that turns bad during the
warm-up still refuses. Move results report `warmup_s`.
Note: ROLA streams frames every ~50 ms; 20 ms is new for the firmware — if moves misbehave, set
`EBO_MOTOR_FRAME_S=0.05`.

## POST /turn — turn by a measured angle (closed loop)

`POST /turn {"deg": 90, "tol": 5, "max_pulses": 10}` (+ = right / clockwise seen from above).

Why: the firmware cannot turn by an arbitrary angle. Measured with an overhead 446 fps camera
(porch repo `tools/turncal`, runs 1–8): every turn pulse ends either as a short **blip** (~7–27°)
or as a **full turn** (right ~50° ±10, left ~61° ±4 on bare floor); 0.12–0.18 s pulses are a coin
flip between the two, very short ones are sometimes lost. Timing alone cannot hit an angle.

How (`app/ebo_turn.py`): still frame → one pulse (through the normal move path: dock guard, warm-up
from idle, 20 ms frames) → wait for a still frame again → measure the yaw between the two still
frames with the robot's own camera → choose the next pulse from what is left (full turn while the
remainder is ≥ 0.8 × the side's full turn, else a blip; a blip is only tried if some outcome could
land within `tol`) → stop within `tol`, or when nothing can improve it, or after `max_pulses`.
While the total is ≤ 65° it is re-measured directly against the first frame (no accumulated error).
Nominal pulse sizes adapt to what the run measures; a lost blip lengthens that side's blip, a blip
that came out as a full turn shortens it (range 0.06–0.14 s — with 20 ms frames + warm-up, short
left blips of 0.06–0.13 s gave small usable steps live, 2026-10-04).

**Settling (fix 2026-10-04):** the first live run measured every pulse one pulse late — right after a
wake/idle the robot stream lags 0.3–1.5 s, so a fixed wait after the pulse grabbed still frames from
*before* the motion (pulse 1 read 0°, the next pulse read both, then a "wrong way" abort; 90° became
~120°). Now the helper's `settle` waits until the picture has **changed** against the before-frame
and is **still again** (≥ 3 frames over ≥ 0.3 s, all newer than the pulse END and the first change),
never earlier than END + `settle_min_s` (0.5 s); if nothing changes until END + `no_motion_s` (1.5 s)
the pulse counts as lost (0°); no settle by END + `settle_max_s` (3 s) → camera error, nothing counted.
Every pulse logs the measured stream latency (`latency_s` from START, `latency_end_s` from END) and
`settle_s`. A wrong-way reading is re-measured once on a fresh still frame before aborting. The
decision logic is `ebo_turn.settle_pick` (pure, shared by the helper and the tests);
`"settle_mode": "still"` restores the old fixed wait.

Vision (`tools/ebo_turn_vision.py`, helper process — needs numpy + OpenCV, set
`EBO_TURN_VISION_PYTHON`): frames from the robot's RTSP stream are reprojected into a **levelled
equirectangular strip** (calibrated fisheye `config/ebo_se_fisheye_1920.json`, camera pitched up
`EBO_TURN_PITCH_DEG`=15°), where a turn about the robot's vertical axis is a pure horizontal shift;
the shift is found by normalised cross-correlation of gradient images. Synthetic tests on real robot
frames: mean error < 0.1°, reliable up to ~70°, larger turns are reported as not ok (then frames
recorded during the motion are chained, else the pulse counts with its nominal size and the result
says `"uncertain": true`).

Result: `{"ok", "target_deg", "turned_deg", "error_deg", "tol_deg", "pulses": [{kind, rx, dur,
measured_deg, method, moved, latency_s, latency_end_s, settle_s}], "uncertain", "reason", "blip_dur"}`. Refused while docked (dock guard). Tuning file:
`EBO_TURN_CONFIG` (JSON, overrides `ebo_turn.DEFAULTS`). Tests: `tests/test_ebo_turn.py` (controller
vs the measured two-mode turn model: 96% ok with instant frames; with a simulated 0.3–1.5 s stream
lag the old fixed wait reproduces the live failure — first pulse ~0° in 40/40 — and settling reaches
94% within 5°), `tests/test_ebo_turn_vision.py`
(needs OpenCV; skipped in the bridge venv).
