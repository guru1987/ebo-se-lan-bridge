# Measured small turns — EBO SE, 2026-10-15

The existing `/move` endpoint produces small left/right turns with zero forward/backward
input. No bridge code change or Pixel connection is required. Both services on .135 are
running. Firmware `control-speed` was set to 10 and read back as 10.

First set `PUT /parameter/control/speed` with `{"value":10}` and require acknowledged=true.
Then use ONE of the following authenticated `POST /move` requests:

| Step | Request body | Measured camera yaw |
| --- | --- | --- |
| Small left | `{"ly":0,"rx":-0.33,"duration":0.10}` | -9.50 degrees; MCP repeat -6.39 |
| Small right | `{"ly":0,"rx":0.33,"duration":0.10}` | +7.25 degrees; MCP repeat +10.75 |
| Medium left | `{"ly":0,"rx":-0.20,"duration":0.10}` | -13.75 degrees |
| Medium right | `{"ly":0,"rx":0.20,"duration":0.10}` | +12.75 degrees |

Each `/move` ends with a zero-axis motor frame. From idle it first sends 0.6 seconds of
zero-axis warmup frames; movement duration excludes that warmup. Keep a settling interval
between steps; calibration used three seconds before taking each after-image.

The 0.33-deflection pair was also measured at 100 ms: -9.50 and +7.25 degrees, respectively.
40 ms at 0.33 produced no motion. Deflection 0.10 at 60/100 ms and deflection 0.20 at 60 ms
also produced no measurable motion. The response is not a simple linear duration/amplitude
law, and reliable five-degree stepping has not been established. Do not label these empirical
presets as exact 5/10/15-degree commands.

Angles are optical yaw estimates from the existing calibrated fisheye strip matcher, with
NCC 0.90-0.94 for moving frames; no depth parallax correction was applied. Nearby objects,
floor friction and battery state can affect results. Repeated trials are too few to establish
statistical tolerances. Independent mechanical translation was not measured, though the
forward/backward command was exactly zero. Existing `/turn` controller defaults were left
unchanged; these presets belong to `/move`.

## MCP observe-turn-observe path

`body_turn_step(direction="left", size="small", look=True)` (or direction="right") sets
firmware speed=10 and requires its acknowledgement, sends one pure-yaw 100-ms /move,
explicitly stops, waits three seconds, then returns a JPEG with the result text. `size="medium"`
uses amplitude 0.20; small uses 0.33. `look=False` omits the camera response.
The tool is registered only for the body's owner, through the existing body.register path.
Its code and tool-sheet entry are installed in the porch source tree on .135 and this workstation.
An already-running brain/MCP process must load the updated source to acquire the new tool;
the live verification used a fresh MCP stdio server, without restarting any brother session.

A real MCP client discovered the tool and called body_look, left step, right step, receiving
valid camera JPEGs after each action. Camera comparison measured -6.39 degrees and +10.75 degrees
with good registration confidence. An earlier 60-ms version returned valid responses but
produced zero camera motion on both calls; that failed trial is why the final tool uses 100 ms.
The tool text reports calibration expectations, not a claimed exact angle measured on every call.

Machine-readable companion: `config/ebo_small_turns_20261015.json`. Raw calibration measurements
and images are saved on the operator workstation in `fourth_body/frida/logs/task7_calibration*`.
