# EBO SE talkback

The bridge supports audio in both directions:

- robot microphone to the bridge through the existing `avRecvAudioData` path;
- browser microphone to the robot speaker through TUTK AV server channel 2.

## Native protocol

The supervisor writes length-prefixed commands to native bridge fd 3:

| Kind | Payload |
| --- | --- |
| `0` | existing MAVLink/RDT control |
| `1` | existing IOCTRL command |
| `2` | start speaker; no payload |
| `3` | nonempty, even-length PCM16 little-endian, 8 kHz mono; maximum 1280 bytes |
| `4` | stop speaker; no payload |

On START the native bridge runs `avServStartEx` asynchronously on IOTC channel 2 and sends
IOCTRL `0x350` over the main AV session. Authentication returns the device's existing AV
token. PCM is converted byte-for-byte like ROLA's `linear2alaw` implementation and sent with
`avSendAudioData`.

The 16-byte frame metadata contains codec `0x008A` little-endian, flag byte `0x0E`, and the
low 32 bits of the wall-clock millisecond timestamp at offsets 12 through 15. STOP sends
IOCTRL `0x351`, stops the speaker AV index, disables channel 2, exits the AV server and joins
the startup worker before the main session can reconnect.

## Web panel

`/ws/talk` is a same-origin WebSocket protected by the panel's Basic authentication when
`EBO_WEB_USER` is configured. It admits one owner, waits for the native `speaker ready`
event, validates frame size and real-time rate, and always stops the native speaker in a
`finally` cleanup path.

The Hold to talk button requests a mono microphone stream, downsamples it in the browser to
8 kHz signed PCM16, and sends audio only while held. Browser microphone capture requires
HTTPS or a trusted `localhost` context.

## Validation status

Offline tests cover ROLA-compatible A-law vectors, exact frame metadata and IOCTRL payloads,
speaker lifecycle/join behavior, control framing, readiness timeout, authentication and
WebSocket-disconnect cleanup. An ARM64 bridge built with Android NDK r30 was then exercised
on a rooted Pixel 3 with the owned ROLA ARM64 libraries: the main session connected, inbound
audio continued, channel 2 reported ready as speaker AV index 1, the paced test frames were
accepted, STOP completed, and the process exited cleanly.

The remaining deployment check is the full container and real browser microphone path on
the selected ARM host. Keep the robot on the floor and supervised during first-run tests.

For a supervised end-to-end check through a running supervisor, load the protected runtime
environment and run `scripts/smoke_websocket_talk.py`. It sends a low-volume 1.6-second tone;
it never sends movement commands.
