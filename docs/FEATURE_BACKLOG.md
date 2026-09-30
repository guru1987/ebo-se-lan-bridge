# Remaining feature backlog

1. **Control-only / low-heat mode — implemented and live validated**
   `POST /video/stop|start` toggles only video. During a live pause, `/ebo_audio`
   delivered 159,992 PCM bytes in five seconds while 161 robot audio frames arrived;
   talkback stayed online, and video start waited for a publishable frame.

2. **On-demand snapshots — protocol implemented, live validation pending**
   `POST /photo` creates a full-HD SD photo. A transient live-frame JPEG endpoint remains optional.

3. **Movement-scoped video — integration pending**
   Video can now be toggled independently; ebo-eye still needs movement/snapshot policy wiring.

4. **Reliable talkback lifecycle — implemented**
   Persistent ownership is handled by ebo-eye through `/ws/talk`.

5. **Audio-volume calibration — implemented in ebo-eye**
   Robot media and call volume stay at 50; software gain owns perceived loudness.

6. **Motor calibration harness — pending**
   External video/audio calibration and calculated thresholds remain future work.

7. **Floor movement validation — pending**
   Validate direction, turning, stop latency, and load behavior with hands-on supervision.

8. **Collision and fall-protection validation — pending live test**
   Parameters echo, but they are not safety guarantees until tested on SE1 hardware.

9. **Patrol task completion — implementation complete, live validation pending**
   Fixed-point and cruise schedules are exposed; live update/delete behavior still needs checking.

10. **Immediate patrol controls — pending**
    Scheduled tasks exist; immediate start/stop commands are not identified yet.

11. **Expanded telemetry — implementation complete, live validation pending**
    IMU, wheel, temperature, camera/MCU, battery, and optional rangefinder fields are decoded.

12. **SD-card media access — implemented and live validated**
    `GET /media` listed five card items and `GET /media/{name}` downloaded an
    existing 486,484-byte JPEG with a valid JPEG signature.

13. **Camera controls — substantially implemented and partly live validated**
    Live-view Auto/High/Medium/Low quality, video start/stop, and day/night state
    reads are validated. Full-HD photo and forced colour/IR still await live validation.

14. **Security-event integration — partial**
    Settings and schedules exist; event history and complete notifications remain pending.

15. **Home Assistant completion — partial**
    Add media, camera/video policy, DND/auto-return, and newly validated telemetry entities.

16. **MCP interface — pending in parallel session**
    Wrap the stable REST/WebSocket surface with strict movement and media bounds.

17. **Runtime device discovery — pending**
    Replace environment-supplied static identity fields with runtime capability queries.

18. **Single-session coordination — partial**
    Manual bridge/app handoff works; ownership display and automatic recovery need refinement.

19. **Deployment and release — deployed, live validation in progress**
    The feature branch is running on the bridge; media listing/download and
    day/night state reads are validated before preparing the upstream contribution.

20. **Battery auto-return switch — implemented, live validation pending**
    Actions use ROLA's type-3 do-not-disturb mechanism; there is no separate auto-return parameter.
