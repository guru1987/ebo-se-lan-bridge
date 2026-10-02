"""
EBO-SE bridge supervisor.

Spawns mediamtx (RTSP/WebRTC/HLS server), ffmpeg (H.265 passthrough, no transcode)
and the native ebo_bridge. Forwards video frames to ffmpeg, parses inbound status
(battery), exposes a small authenticated control API + web panel, and (optionally)
publishes Home Assistant entities over MQTT.

Required env:  EBO_LICENSE EBO_UID EBO_AUTHKEY EBO_IDENTITY EBO_TOKEN
Stream auth:   EBO_STREAM_USER EBO_STREAM_PASS         (mediamtx)
Web panel:     EBO_WEB_USER EBO_WEB_PASS               (HTTP basic; disabled if unset)
MQTT:          EBO_MQTT_HOST [EBO_MQTT_PORT EBO_MQTT_USER EBO_MQTT_PASS]
Other:         EBO_DIR (/opt/ebo)  EBO_PORT (8000)  EBO_BIND (127.0.0.1)
"""
import os, json, math, time, struct, threading, subprocess, secrets, socket, base64, urllib.request, urllib.error, http.cookiejar, tempfile
from dataclasses import replace
from pathlib import Path
from fastapi import FastAPI, Depends, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from starlette.background import BackgroundTask
from starlette.concurrency import run_in_threadpool
from starlette.requests import HTTPConnection
from pydantic import BaseModel, Field
import uvicorn

from ebo_mavlink import (FrameBuilder, MAVLinkV1Stream, TimedTask,
                         decode_telemetry, parse_parameter_value,
                         parse_file_server_info, parse_task_ack, parse_timed_task)
from ebo_media import EboMediaClient, MediaProtocolError

EBO_DIR = os.environ.get("EBO_DIR", "/opt/ebo")
PORT = int(os.environ.get("EBO_PORT", "8000"))
# Bind the web panel to loopback by default: it is reached through the Cloudflare
# tunnel (or a reverse proxy), not exposed directly on the LAN. Set EBO_BIND=0.0.0.0
# to expose it on the LAN instead.
BIND_HOST = os.environ.get("EBO_BIND", "127.0.0.1")
# Robot audio (listen-only). EBO_AUDIO controls whether it is also muxed into the legacy
# combined /ebo path. The independent /ebo_audio path is always published, so hearing remains
# available when video is stopped (G.711 8 kHz mono; use EBO_AUDIO_FMT=alaw if distorted).
AUDIO_ON = os.environ.get("EBO_AUDIO", "0") == "1"
AUDIO_FMT = os.environ.get("EBO_AUDIO_FMT", "mulaw")   # mulaw | alaw
AUDIO_RATE = os.environ.get("EBO_AUDIO_RATE", "8000")
# Audio processing (ffmpeg -af). Default "none" = raw G.711 passthrough (PCMU), lowest latency —
# this is the smooth, real-time path. Setting a filter chain enables denoise/volume but transcodes
# to Opus, which adds latency (the player then delays video to stay A/V-synced). Opt-in only, e.g.:
#   EBO_AUDIO_FILTER=highpass=f=200,afftdn=nr=20,dynaudnorm=g=11:m=15,alimiter=limit=0.9
AUDIO_FILTER = os.environ.get("EBO_AUDIO_FILTER", "none")
RTSP_PATH = "ebo"
RTSP_AUDIO_PATH = "ebo_audio"
VIDEO_READY_TIMEOUT = float(os.environ.get("EBO_VIDEO_READY_TIMEOUT", "12"))
VIDEO_QUALITY_IOCTL = int(os.environ.get("EBO_VIDEO_QUALITY_IOCTL", "0x990a"), 0)
if VIDEO_QUALITY_IOCTL not in (0x9906, 0x990a):
    raise ValueError("EBO_VIDEO_QUALITY_IOCTL must be 0x9906 or 0x990a")
VIDEO_QUALITY_PROFILES = {
    "auto": (0, 0, "adaptive"),
    "high": (1, 4096, "1920x1080"),
    "medium": (2, 1500, "848x480"),
    "low": (3, 768, "848x480"),
}
STREAM_USER = os.environ.get("EBO_STREAM_USER", "ebo")
STREAM_PASS = os.environ.get("EBO_STREAM_PASS", "")
WEB_USER = os.environ.get("EBO_WEB_USER")
WEB_PASS = os.environ.get("EBO_WEB_PASS")

def _has_param_set(data, codec):
    """True if an Annex-B frame carries codec parameter sets, i.e. is a clean
    point for ffmpeg to start: HEVC VPS(32)/SPS(33), H.264 SPS(7)."""
    i, n = 0, len(data)
    while i + 4 < n:
        if data[i] == 0 and data[i+1] == 0 and (data[i+2] == 1 or (data[i+2] == 0 and i+3 < n and data[i+3] == 1)):
            j = i + (3 if data[i+2] == 1 else 4)
            if j < n:
                if codec == 80:                       # HEVC
                    t = (data[j] >> 1) & 0x3F
                    if t == 32 or t == 33:
                        return True
                else:                                 # H.264
                    if (data[j] & 0x1F) == 7:
                        return True
            i = j
        else:
            i += 1
    return False

# ---------------- MAVLink builders (control over RDT) ----------------
def mavlink_crc(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        t = b ^ (crc & 0xFF); t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    t = crc_extra ^ (crc & 0xFF); t = (t ^ (t << 4)) & 0xFF
    crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc

def _frame(msgid, payload, crc_extra):
    hdr = bytes([len(payload), 0, 0, 0, msgid])
    return bytes([0xfe]) + hdr + payload + struct.pack('<H', mavlink_crc(hdr + payload, crc_extra))

def motor_frame(ly=0.0, rx=0.0, lx=0.0, ry=0.0, buttons=0):
    # Public bridge convention: positive ly is forward. ROLA's native vertical
    # joystick coordinate has the opposite sign, so msg 202 carries -ly.
    return _frame(202, struct.pack('<ffff', lx, -ly, rx, ry) + bytes([buttons & 0xFF]), 211)

def param_set_frame(group, key, value, ptype=11):
    pid = f"{group}-{key}".encode()[:32].ljust(32, b'\x00')
    return _frame(229, struct.pack('<f', value) + bytes([255, 3]) + pid + bytes([ptype]), 208)

def command_frame(command):
    return _frame(200, struct.pack('<H', command) + bytes([255, 1]), 196)

CMD_DOCK = 40154
# ROLA: direct live controls plus SEDeviceInfoFragment -> AirAutoFragment ->
# LiveViewModel.r(). These are firmware-owned, self-terminating motions. Keep
# the public names explicit: accepting arbitrary command IDs here would bypass
# the API's movement safety boundary. "rocket" is the familiar name for
# ROLA's "Rush Forward" control; both names map to the same command.
SPECIAL_MOTION_COMMANDS = {
    "big_circle": 40115, "rotate": 40115, "donut": 40115,
    "straight_line": 40116, "rush_forward": 40116, "rocket": 40116,
    "swing": 40129, "shake": 40129,
    "spin": 40164, "race": 40165, "excited": 40166,
    "relaxed": 40167, "little_monster": 40168,
}
MAVLINK = FrameBuilder()

# Device info (captured; for a community release these would be queried at runtime via MAVLink)
DEVICE_INFO = {
    "name": os.environ.get("EBO_NAME", "EBO-SE"),
    "model": "EBO SE",
    "sn": os.environ.get("EBO_SN", ""),
    "fsn": os.environ.get("EBO_FSN", ""),
    "mcu_version": os.environ.get("EBO_MCU", ""),
    "camera_version": os.environ.get("EBO_CAMFW", ""),
    "uid": os.environ.get("EBO_UID", ""),
    "wifi_ssid": os.environ.get("EBO_SSID", ""),
    "ip": os.environ.get("EBO_ROBOT_IP", ""),
    "mac": os.environ.get("EBO_MAC", ""),
}

PARAM_TOGGLES = {
    "eyes_on": ("display", "enable", 1.0), "eyes_off": ("display", "enable", 0.0),
    "night_auto": ("video", "night_vision", 0.0),
    "day": ("video", "night_vision", 1.0), "night_off": ("video", "night_vision", 1.0),
    "night_on": ("video", "night_vision", 2.0),
    "avoid_on": ("control", "auto_avoidance", 1.0), "avoid_off": ("control", "auto_avoidance", 0.0),
    "fall_on": ("control", "fallarrest", 1.0), "fall_off": ("control", "fallarrest", 0.0),
    "patrol_on": ("security_patrol", "enable", 1.0), "patrol_off": ("security_patrol", "enable", 0.0),
    "sleep": ("power", "sleep", 1.0), "wake": ("power", "sleep", 0.0),
}

# Parameters verified in the ROLA APK for EBO SE.  The type is MAVLink's
# parameter type value; min/max are UI/API safeguards, not protocol limits.
PARAMETERS = {
    "display-enable": {"type": 11, "min": 0, "max": 1},
    "video-night_vision": {"type": 6, "min": 0, "max": 2},
    "control-auto_avoidance": {"type": 11, "min": 0, "max": 1},
    "control-fallarrest": {"type": 11, "min": 0, "max": 1},
    "control-speed": {"type": 6, "min": 0, "max": 100},
    "audio-volume": {"type": 6, "min": 0, "max": 100},
    "audio-call_volume": {"type": 6, "min": 0, "max": 100},
    "security_patrol-enable": {"type": 11, "min": 0, "max": 1},
    "security_fixed-enable": {"type": 11, "min": 0, "max": 1},
    "security_fixed-notification": {"type": 11, "min": 0, "max": 1},
    "security_fixed-notify_interval": {"type": 6, "min": -1, "max": 3600},
    "security_fixed-range_point1_x": {"type": 6, "min": 0, "max": 100},
    "security_fixed-range_point1_y": {"type": 6, "min": 0, "max": 100},
    "security_fixed-range_point2_x": {"type": 6, "min": 0, "max": 100},
    "security_fixed-range_point2_y": {"type": 6, "min": 0, "max": 100},
    "security_fixed-sensitivity": {"type": 6, "min": 0, "max": 4},
    "power-sleep": {"type": 11, "min": 0, "max": 1},
}

def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]; s.close(); return ip
    except Exception:
        return "127.0.0.1"


class Bridge:
    def __init__(self):
        self.frame_count = 0
        self.last_frame = 0.0   # time.time() of the last video frame (for awake/sleep detection)
        self.audio_codec = None # codec_id of inbound audio frames (discovered at runtime)
        self.audio_flags = None # TUTK audio flags (sample rate / bits / channel)
        self.audio_count = 0
        self.last_audio = 0.0
        self.connected = False
        self.codec_name = None
        self.battery = -1
        self.charge = -1   # 0 = unplugged, 1 = charging/docked
        self.on_status = None   # callback(battery, charge) for MQTT
        self.on_state = None    # callback(kind, name, value) for MQTT/other integrations
        self.parameters = {}
        self.parameter_meta = {}
        self.telemetry = {}
        self.telemetry_updated = {}   # name -> unix time of the last frame that carried it
        self.raw_ids = {}             # MAVLink id -> {"count", "last", "len"} (diagnostics)
        self.timed_tasks = {1: {}, 2: {}, 3: {}}
        self._task_list_counts = {1: None, 2: None, 3: None}
        self.last_task_ack = None
        self.file_server_info = None
        self.video_enabled = True
        self.video_quality = None
        self.video_quality_updated = 0.0
        self._published_frame_count = 0
        self._last_published_frame = 0.0
        self._video_start_publish_count = -1
        self._media_lock = threading.Lock()
        self._autoreturn_saved_task = None
        self._mav_stream = MAVLinkV1Stream()
        self._state_condition = threading.Condition()
        self._ctrl_r, self._ctrl_w = os.pipe()
        self._a_r, self._a_w = os.pipe()    # robot audio -> ffmpeg second input
        os.set_inheritable(self._a_r, True)
        os.set_blocking(self._a_w, False)   # never block: drop audio if ffmpeg lags
        self._ao_r, self._ao_w = os.pipe()  # robot audio -> independent audio-only ffmpeg
        os.set_inheritable(self._ao_r, True)
        os.set_blocking(self._ao_w, False)
        self._a_buf = bytearray()           # jitter buffer for robot audio
        self._a_lock = threading.Lock()
        self._ctrl_lock = threading.Lock()  # keep fd3 frames atomic across HTTP/WebSocket threads
        self._speaker_ready = threading.Event()
        self._speaker_error = None
        self._speaker_state_lock = threading.Lock()
        self._initial_refresh_started = False
        self._running = True
        self._ff_primed = False   # becomes True once ffmpeg has been fed an HEVC keyframe
        self.paused = False       # when True the native bridge is stopped, freeing the robot for the app
        self._start_mediamtx()
        time.sleep(0.6)
        self._start_ffmpeg()
        self._start_audio_ffmpeg()
        self._start_bridge()
        threading.Thread(target=self._read_frames, daemon=True).start()
        threading.Thread(target=self._log_stderr, args=(self.proc,), daemon=True).start()
        threading.Thread(target=self._audio_feeder, daemon=True).start()

    def _audio_feeder(self):
        # Emit smooth real-time G.711 to the independent audio publisher and, when enabled, the
        # legacy combined publisher. Silence keeps each RTSP path continuously readable.
        # Silence byte: 0xFF µ-law / 0xD5 A-law.
        rate = int(AUDIO_RATE); chunk = max(80, rate // 50); interval = chunk / rate   # ~20 ms
        silence = (b"\xff" if AUDIO_FMT == "mulaw" else b"\xd5") * chunk
        next_t = time.monotonic()
        last_audio_restart = 0.0
        while self._running:
            next_t += interval
            with self._a_lock:
                if len(self._a_buf) >= chunk:
                    out = bytes(self._a_buf[:chunk]); del self._a_buf[:chunk]
                elif self._a_buf:
                    out = bytes(self._a_buf) + silence[len(self._a_buf):]; self._a_buf.clear()
                else:
                    out = silence
            if self.audio_ff.poll() is not None and time.monotonic() - last_audio_restart >= 2.0:
                last_audio_restart = time.monotonic()
                self._restart_audio_ffmpeg()
            targets = (self._ao_w, self._a_w) if AUDIO_ON else (self._ao_w,)
            for target in targets:
                try: os.write(target, out)
                except (BlockingIOError, BrokenPipeError, OSError): pass
            d = next_t - time.monotonic()
            if d > 0: time.sleep(d)
            else: next_t = time.monotonic()

    def is_awake(self):
        # The camera only streams when the robot is awake; treat "a video frame within
        # the last few seconds" as awake, otherwise the robot is sleeping / camera off.
        return (time.time() - self.last_frame) < 6.0

    def _start_mediamtx(self):
        self.mtx = subprocess.Popen([os.path.join(EBO_DIR, "mediamtx"), os.path.join(EBO_DIR, "mediamtx.yml")],
                                    cwd=EBO_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[supervisor] mediamtx pid", self.mtx.pid, flush=True)

    def _start_ffmpeg(self):
        # MediaMTX permits anonymous publishing only from loopback. Keep reader
        # credentials out of ffmpeg's process arguments and /proc command line.
        url = f"rtsp://127.0.0.1:8554/{RTSP_PATH}"
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning",
               "-fflags", "nobuffer", "-flags", "low_delay",
               "-analyzeduration", "500000", "-probesize", "1000000",
               "-thread_queue_size", "256", "-f", "hevc", "-i", "pipe:0"]
        pass_fds = ()
        if AUDIO_ON:
            # second input: raw G.711 from the robot, passed through as PCMU/PCMA (no transcode).
            # -max_interleave_delta 0 = emit packets immediately, never hold video to wait for
            # audio interleaving (that buffering was adding seconds of latency to both).
            if AUDIO_FILTER and AUDIO_FILTER.lower() != "none":
                aout = ["-af", AUDIO_FILTER, "-c:a", "libopus", "-application", "voip",
                        "-b:a", "24k", "-ar", "48000", "-ac", "1"]
            else:
                aout = ["-c:a", "copy"]   # raw G.711 passthrough (PCMU/PCMA), no processing
            cmd += ["-use_wallclock_as_timestamps", "1", "-thread_queue_size", "256",
                    "-f", AUDIO_FMT, "-ar", AUDIO_RATE, "-ac", "1", "-i", f"pipe:{self._a_r}",
                    "-map", "0:v", "-map", "1:a", "-c:v", "copy"] + aout + \
                   ["-max_interleave_delta", "0"]
            pass_fds = (self._a_r,)
        else:
            cmd += ["-c:v", "copy", "-an"]
        cmd += ["-f", "rtsp", "-rtsp_transport", "tcp", url]
        self.ff = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, pass_fds=pass_fds)
        print("[supervisor] ffmpeg pid", self.ff.pid, flush=True)

    def _start_audio_ffmpeg(self):
        """Publish robot microphone audio independently of the video muxer."""
        url = f"rtsp://127.0.0.1:8554/{RTSP_AUDIO_PATH}"
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning",
               "-fflags", "nobuffer", "-flags", "low_delay",
               "-use_wallclock_as_timestamps", "1", "-thread_queue_size", "256",
               "-f", AUDIO_FMT, "-ar", AUDIO_RATE, "-ac", "1", "-i", f"pipe:{self._ao_r}",
               "-map", "0:a", "-c:a", "copy", "-vn",
               "-f", "rtsp", "-rtsp_transport", "tcp", url]
        self.audio_ff = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                         pass_fds=(self._ao_r,))
        print("[supervisor] audio ffmpeg pid", self.audio_ff.pid, flush=True)

    def _restart_audio_ffmpeg(self):
        try: self.audio_ff.kill()
        except Exception: pass
        self._start_audio_ffmpeg()
        print("[supervisor] audio ffmpeg restarted", flush=True)

    def _restart_ffmpeg(self):
        # ffmpeg died (e.g. broken pipe). Restart it and re-sync at the next keyframe.
        try: self.ff.stdin.close()
        except Exception: pass
        try: self.ff.kill()
        except Exception: pass
        self._ff_primed = False
        self._start_ffmpeg()
        print("[supervisor] ffmpeg restarted", flush=True)

    def _start_bridge(self):
        linker = os.path.join(EBO_DIR, "bionic", "linker")
        bridge_name = os.environ.get("EBO_BRIDGE_BIN", "ebo_bridge")
        if os.path.basename(bridge_name) != bridge_name:
            raise ValueError("EBO_BRIDGE_BIN must be a filename")
        bridge = os.path.abspath(os.path.join(EBO_DIR, bridge_name))
        qemu = os.environ.get("EBO_QEMU")
        command = [qemu, linker, bridge] if qemu else [linker, bridge]
        env = dict(os.environ)
        env["EBO_LIB_DIR"] = os.path.join(EBO_DIR, "lib")
        env["EBO_IOCTL9930"] = os.path.join(EBO_DIR, "ioctl9930.bin")
        env["LD_LIBRARY_PATH"] = os.path.join(EBO_DIR, "bionic") + ":" + os.path.join(EBO_DIR, "lib")
        self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    env=env, pass_fds=(self._ctrl_r,),
                                    preexec_fn=lambda: os.dup2(self._ctrl_r, 3))
        with self._speaker_state_lock:
            self._speaker_error = None
            self._speaker_ready.clear()
        print("[supervisor] bridge pid", self.proc.pid, flush=True)

    def pause_bridge(self):
        # Stop the native bridge -> releases the robot's single P2P session so the Enabot app
        # can connect. ffmpeg/mediamtx keep running; video just stops until resumed.
        if self.paused:
            return
        self.paused = True
        self.connected = False
        try: self.speaker_stop()
        except Exception: pass
        print("[supervisor] pausing bridge (releasing robot for the app)", flush=True)
        try: self.proc.terminate()
        except Exception: pass
        try: self.proc.wait(timeout=3)
        except Exception:
            try: self.proc.kill()
            except Exception: pass

    def resume_bridge(self):
        # Reconnect to the robot (will fail/retry while the native app still holds the session).
        if not self.paused:
            return
        self.paused = False
        self._ff_primed = False
        self._initial_refresh_started = False
        print("[supervisor] resuming bridge", flush=True)
        self._start_bridge()
        threading.Thread(target=self._read_frames, daemon=True).start()
        threading.Thread(target=self._log_stderr, args=(self.proc,), daemon=True).start()

    def _log_stderr(self, proc):
        for line in iter(proc.stderr.readline, b""):
            s = line.decode(errors="replace").rstrip()
            print("[bridge]", s, flush=True)
            if "connected" in s:
                self.connected = True
                if not self.video_enabled:
                    try: self.send_ioctl(0x02ff, bytes(8))
                    except Exception: pass
                if not self._initial_refresh_started:
                    self._initial_refresh_started = True
                    threading.Thread(target=self.refresh_parameters, daemon=True).start()
            if "speaker ready " in s:
                with self._speaker_state_lock:
                    self._speaker_error = None
                    self._speaker_ready.set()
            elif "avServStartEx speaker err=" in s:
                with self._speaker_state_lock:
                    self._speaker_error = s.rsplit("=", 1)[-1]
                    self._speaker_ready.set()
        if proc is self.proc:
            self.connected = False
            with self._speaker_state_lock:
                if not self._speaker_ready.is_set():
                    self._speaker_error = "bridge disconnected"
                    self._speaker_ready.set()

    def _read_frames(self):
        f = self.proc.stdout
        while self._running:
            hdr = f.read(5)
            if len(hdr) < 5:
                break
            n = struct.unpack("<I", hdr[:4])[0]; codec = hdr[4]
            data = f.read(n)
            if len(data) < n:
                break
            if codec == 0xFF:
                self._parse_status(data)   # inbound MAVLink (battery/status)
                continue
            if codec == 0xA0:              # audio frame: [codec_id:1][flags:1][data]
                if len(data) >= 2:
                    self.audio_codec, self.audio_flags = data[0], data[1]
                    self.audio_count += 1
                    self.last_audio = time.time()
                    if len(data) > 2:
                        with self._a_lock:
                            self._a_buf += data[2:]
                            if len(self._a_buf) > 1600:      # cap latency (~200ms): drop oldest
                                del self._a_buf[:len(self._a_buf) - 1600]
                continue
            with self._state_condition:
                self.frame_count += 1
                self.last_frame = time.time()
            self.codec_name = "hevc" if codec == 80 else ("h264" if codec == 78 else f"codec{codec}")
            # ffmpeg must start at a keyframe: skip frames until one carries the
            # parameter sets (HEVC VPS/SPS, H.264 SPS), otherwise it can't determine
            # the stream dimensions and exits ("dimensions not set").
            if not self._ff_primed:
                if not _has_param_set(data, codec):
                    continue
                self._ff_primed = True
            try:
                self.ff.stdin.write(data); self.ff.stdin.flush()
                with self._state_condition:
                    self._published_frame_count += 1
                    self._last_published_frame = time.time()
                    self._state_condition.notify_all()
            except Exception:
                self._restart_ffmpeg()

    def _parse_status(self, u):
        for frame in self._mav_stream.feed(u):
            changed = []
            raw = getattr(self, "raw_ids", None)
            if raw is None: raw = self.raw_ids = {}
            rec = raw.setdefault(frame.message_id, {"count": 0, "last": 0.0, "len": 0})
            rec["count"] += 1; rec["last"] = time.time(); rec["len"] = len(frame.payload)
            try:
                if frame.message_id == 228:
                    value = parse_parameter_value(frame.payload)
                    with self._state_condition:
                        self.parameters[value.name] = value.value
                        self.parameter_meta[value.name] = {
                            "type": value.parameter_type, "count": value.count,
                            "index": value.index, "updated": time.time()}
                        self._state_condition.notify_all()
                    changed.append(("parameter", value.name, value.value))
                elif frame.message_id == 154:
                    task = parse_timed_task(frame.payload)
                    with self._state_condition:
                        self._task_list_counts[task.task_type] = task.count
                        # Firmware represents an empty list with one zeroed ITEM
                        # whose count is 0.  It is a sentinel, not task id 0.
                        if task.count:
                            self.timed_tasks[task.task_type][task.task_id] = task.as_dict()
                        self._state_condition.notify_all()
                    if task.count:
                        changed.append(("task", str(task.task_id), task.as_dict()))
                elif frame.message_id == 157:
                    ack = parse_task_ack(frame.payload)
                    with self._state_condition:
                        self.last_task_ack = {**ack, "updated": time.time()}
                        self._state_condition.notify_all()
                    changed.append(("task_ack", str(ack["task_id"]), ack))
                elif frame.message_id == 42:
                    info = parse_file_server_info(frame.payload)
                    with self._state_condition:
                        self.file_server_info = {**info, "updated": time.time()}
                        self._state_condition.notify_all()
                telemetry = decode_telemetry(frame)
                if telemetry:
                    with self._state_condition:
                        self.telemetry.update(telemetry)
                        now = time.time()
                        upd = getattr(self, "telemetry_updated", None)
                        if upd is None: upd = self.telemetry_updated = {}
                        for name in telemetry: upd[name] = now
                        self._state_condition.notify_all()
                    for name, value in telemetry.items():
                        changed.append(("telemetry", name, value))
                    battery = telemetry.get("battery")
                    if battery:
                        bp, cs = battery["percent"], battery["charge_status"]
                        if bp != self.battery or cs != self.charge:
                            self.battery, self.charge = bp, cs
                            if self.on_status:
                                try: self.on_status(bp, cs)
                                except Exception: pass
            except (ValueError, struct.error) as exc:
                print(f"[supervisor] ignored malformed MAVLink {frame.message_id}: {exc}", flush=True)
            if self.on_state:
                for kind, name, value in changed:
                    try: self.on_state(kind, name, value)
                    except Exception: pass

    def read_parameter(self, group: str, key: str, timeout: float = 1.5):
        name = f"{group}-{key}"
        before = self.parameter_meta.get(name, {}).get("updated", 0)
        self.send_rdt(MAVLINK.param_request(group, key))
        end = time.monotonic() + timeout
        with self._state_condition:
            while self.parameter_meta.get(name, {}).get("updated", 0) <= before:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self._state_condition.wait(remaining)
        return self.parameters.get(name)

    def set_parameter(self, group: str, key: str, value: float, timeout: float = 1.5):
        name = f"{group}-{key}"
        spec = PARAMETERS.get(name)
        if spec is None:
            raise ValueError("parameter is not in the EBO SE allowlist")
        value = float(value)
        if not spec["min"] <= value <= spec["max"]:
            raise ValueError(f"value must be {spec['min']}..{spec['max']}")
        if name == "security_fixed-sensitivity" and value not in (0, 2, 4):
            raise ValueError("security sensitivity must be 0, 2, or 4")
        if name == "security_fixed-notify_interval" and not (
                value in (-1, 0) or (60 <= value <= 3600 and value % 60 == 0)):
            raise ValueError("notification interval must be -1, 0, or whole minutes up to 3600 seconds")
        before = self.parameter_meta.get(name, {}).get("updated", 0)
        self.send_rdt(MAVLINK.param_set(group, key, value, spec["type"]))
        end = time.monotonic() + timeout
        with self._state_condition:
            while self.parameter_meta.get(name, {}).get("updated", 0) <= before:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self._state_condition.wait(remaining)
        echoed = self.parameters.get(name)
        return {"requested": value, "value": echoed, "acknowledged": echoed is not None and abs(echoed - value) < 0.001}

    def request_timed_tasks(self, task_type: int, timeout: float = 1.5):
        if task_type not in (1, 2, 3):
            raise ValueError("task type must be 1 (fixed point), 2 (cruise), or 3 (do not disturb)")
        with self._state_condition:
            self.timed_tasks[task_type] = {}
            self._task_list_counts[task_type] = None
        self.send_rdt(MAVLINK.timed_task_list(task_type))
        end = time.monotonic() + timeout
        with self._state_condition:
            while time.monotonic() < end:
                tasks = self.timed_tasks[task_type]
                expected = self._task_list_counts[task_type]
                if expected is not None and len(tasks) >= expected:
                    break
                self._state_condition.wait(max(0, end - time.monotonic()))
            return list(self.timed_tasks[task_type].values())

    def set_timed_task(self, task: TimedTask, timeout: float = 1.5):
        task.validate()
        before = self.last_task_ack.get("updated", 0) if self.last_task_ack else 0
        self.send_rdt(MAVLINK.timed_task_set(task))
        end = time.monotonic() + timeout
        with self._state_condition:
            while time.monotonic() < end:
                ack = self.last_task_ack
                if (ack and ack.get("updated", 0) > before and
                        ack.get("task_id") == task.task_id and ack.get("task_type") == task.task_type):
                    return ack
                self._state_condition.wait(max(0, end - time.monotonic()))
        return None

    def set_auto_return(self, enabled: bool):
        tasks = self.request_timed_tasks(3)
        current = TimedTask(**tasks[0]) if tasks else TimedTask(task_type=3, weekdays=0x7f)
        if not enabled:
            self._autoreturn_saved_task = current
            requested = replace(current, enabled=True, weekdays=0x7f,
                                start_hour=0, start_minute=0, end_hour=23, end_minute=59)
        else:
            saved = self._autoreturn_saved_task
            requested = replace(saved if saved else current, enabled=False)
            self._autoreturn_saved_task = None
        ack = self.set_timed_task(requested)
        if not ack or ack.get("result") != 0:
            raise TimeoutError("robot did not acknowledge do-not-disturb task")
        return {"auto_return": enabled, "do_not_disturb": requested.as_dict(), "ack": ack}

    def refresh_parameters(self, names=None, timeout: float = 0.5):
        """Read supported settings without changing robot state."""
        selected = names or PARAMETERS.keys()
        values = {}
        for name in selected:
            if name not in PARAMETERS or "-" not in name:
                continue
            group, key = name.split("-", 1)
            value = self.read_parameter(group, key, timeout)
            if value is not None:
                values[name] = value
        return values

    def _send_command(self, kind: int, data: bytes = b""):
        if not 0 <= kind <= 255:
            raise ValueError("invalid command kind")
        payload = bytes((kind,)) + data
        framed = struct.pack("<I", len(payload)) + payload
        with self._ctrl_lock:
            view = memoryview(framed)
            while view:
                written = os.write(self._ctrl_w, view)
                if written <= 0:
                    raise BrokenPipeError("bridge control pipe closed")
                view = view[written:]

    def send_rdt(self, mavlink: bytes):
        self._send_command(0, mavlink)

    def send_ioctl(self, io_type: int, data: bytes = b""):
        self._send_command(1, struct.pack("<H", io_type) + data)

    def video_ready(self):
        with self._state_condition:
            return (self.video_enabled and
                    self._published_frame_count > self._video_start_publish_count and
                    time.time() - self._last_published_frame < 2.0)

    def set_video(self, enabled: bool, timeout: float = VIDEO_READY_TIMEOUT):
        with self._state_condition:
            before = self._published_frame_count
            previous = (self.video_enabled, self._video_start_publish_count, self._ff_primed)
            self.video_enabled = enabled
            self._video_start_publish_count = before
            if enabled:
                self._ff_primed = False
        try:
            self.send_ioctl(0x01ff if enabled else 0x02ff, bytes(8))
        except Exception:
            with self._state_condition:
                self.video_enabled, self._video_start_publish_count, self._ff_primed = previous
            raise
        if not enabled:
            return False
        with self._state_condition:
            deadline = time.monotonic() + timeout
            while not self.video_ready():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._state_condition.wait(remaining)
            return True

    def set_video_quality(self, level: str):
        level = str(level).strip().lower()
        try:
            wire_level, bitrate_kbps, resolution = VIDEO_QUALITY_PROFILES[level]
        except KeyError as exc:
            raise ValueError("level must be auto, high, medium, or low") from exc
        # ROLA uses three little-endian uint32 values: level, bitrate target,
        # and an enable/apply flag. The robot persists the selected profile.
        self.send_ioctl(VIDEO_QUALITY_IOCTL, struct.pack("<III", wire_level, bitrate_kbps, 1))
        self.video_quality = level
        self.video_quality_updated = time.time()
        return {
            "accepted": True,
            "level": level,
            "target_resolution": resolution,
            "target_bitrate_kbps": bitrate_kbps or None,
            "ioctrl": hex(VIDEO_QUALITY_IOCTL),
        }

    def open_media_client(self):
        with self._state_condition:
            before = (self.file_server_info or {}).get("updated", 0)
        info = None
        for _ in range(3):
            self.send_rdt(MAVLINK.file_server_open())
            end = time.monotonic() + 3.5
            with self._state_condition:
                while time.monotonic() < end:
                    candidate = self.file_server_info
                    if candidate and candidate.get("updated", 0) > before:
                        before = candidate["updated"]
                        if candidate["status"] == 0:
                            info = candidate
                            break
                    self._state_condition.wait(max(0, end - time.monotonic()))
            if info:
                break
        if not info:
            raise TimeoutError("robot did not open its media file server")
        return EboMediaClient(info["host"], info["port"], info["password"])

    def speaker_start(self, timeout: float = 12.0):
        if self.paused or not self.proc or self.proc.poll() is not None:
            raise RuntimeError("bridge is not running")
        with self._speaker_state_lock:
            self._speaker_error = None
            self._speaker_ready.clear()
        self._send_command(2)
        if not self._speaker_ready.wait(timeout):
            try: self._send_command(4)
            except Exception: pass
            raise TimeoutError("speaker start timed out")
        with self._speaker_state_lock:
            error = self._speaker_error
        if error:
            try: self._send_command(4)
            except Exception: pass
            raise RuntimeError(f"speaker start failed ({error})")

    def speaker_send_pcm(self, pcm16le: bytes):
        if not pcm16le or len(pcm16le) > 1280 or len(pcm16le) % 2:
            raise ValueError("PCM must be even-length, nonempty, and at most 1280 bytes")
        self._send_command(3, pcm16le)

    def speaker_stop(self):
        try:
            self._send_command(4)
        finally:
            with self._speaker_state_lock:
                self._speaker_error = None
                self._speaker_ready.clear()


# ---------------- actions (shared by REST + MQTT) ----------------
# Dock pre-wake: the firmware dock routine can finish in a few seconds, while a paused stream needs
# ~5-7 s until the first depth reading - the dock watch was blind for nearly the whole routine
# (2026-10-02: 3 readings, 63 blind polls in 6.8 s). So before CMD_DOCK the guard wakes the stream and
# waits up to EBO_DOCK_PREWAKE_S for a fresh, floor-scaled reading; it never refuses to dock.
# 0 = off. The result is in the /action/dock response ("prewake") and in the dock watch state.
DOCK_PREWAKE_S = float(os.environ.get("EBO_DOCK_PREWAKE_S", "8"))
_last_dock_prewake = None


def do_action(name: str) -> bool:
    global _last_dock_prewake
    if name == "dock":
        _last_dock_prewake = None
        if guard is not None and DOCK_PREWAKE_S > 0:
            try:
                _last_dock_prewake = guard.wait_reading(DOCK_PREWAKE_S)
            except Exception as e:   # never block docking on the pre-wake
                _last_dock_prewake = {"error": type(e).__name__}
        # pre-dock check: already blocked before the routine starts (wide free < stop distance)?
        # Then sidestep first (same recovery as after a STOP, counts as an attempt) instead of
        # letting the firmware drive straight into it.
        if dockwatch is not None and DOCKWATCH_RECOVER and bridge.charge != 1:
            pre = dockwatch.pre_check()
            if pre is not None:
                dockwatch.start_blocked(prewake=_last_dock_prewake, pre=pre)
                return True
        for _ in range(5): bridge.send_rdt(command_frame(CMD_DOCK)); time.sleep(0.15)
        if dockwatch is not None:
            dockwatch.start(prewake=_last_dock_prewake)
    elif name in SPECIAL_MOTION_COMMANDS:
        # ROLA sends exactly one COMMAND_MINI frame for a selected motion.
        bridge.send_rdt(command_frame(SPECIAL_MOTION_COMMANDS[name]))
    elif name == "undock":
        do_undock()
    elif name == "autoreturn_off":
        bridge.set_auto_return(False)
    elif name == "autoreturn_on":
        bridge.set_auto_return(True)
    elif name in PARAM_TOGGLES:
        g, k, v = PARAM_TOGGLES[name]
        bridge.set_parameter(g, k, v)
    else:
        return False
    return True

# ---------------- forward-move depth guard (opt-in) ----------------
# EBO_DEPTH_GUARD_URL = a depth service's live endpoint (ebo-depth GET /depth/live: JSON
# {"reading": {"free_ahead_m", "free_ahead_kind", "frame_t", ...} | null, "running"}; each
# poll also keeps its stream decoder running). While a FORWARD move runs (ly > 0, /move or
# /drive) a thread polls it every GUARD_POLL_S and stops the robot at once when:
#   free_ahead_m < stop_free_m, free_ahead_m is null (no metric scale / unknown), the view is
#   blocked (free 0), or the depth is down / has no reading / is older than GUARD_STALE_S.
# A forward /move first waits (<= EBO_GUARD_START_WAIT_S) for one fresh good reading and does
# not move without it. Turning (ly == 0) and backing (ly < 0) are never blocked.
# stop_free_m: key of the JSON file EBO_GUARD_CONFIG (ebo-eye's body_config.json, which
# validates it 0.05..1.5), re-read when the file changes; missing/invalid -> 0.25.
# No fresh reading (depth decoder idle/not running, no reading yet, stale - typically because
# ebo-eye paused the video while idle): a forward move first wakes the stream (DepthGuard.wake:
# ebo-eye POST EBO_GUARD_VIDEO_KEEP_URL, then the bridge's own video start if still not ready;
# the guard's /depth/live polls keep the depth decoder running), then waits until the stream is
# up (video publishing + depth decoder running) and EBO_GUARD_WAKE_WAIT_S more for a fresh
# reading (at most EBO_GUARD_WAKE_MAX_S after the wake). After a wake the guard decides only on a
# fresh reading WITH a floor scale (free_ahead_m not null): the first readings after a wake often
# have none yet (camera settling, no floor fit), so once fresh readings flow it waits up to
# EBO_GUARD_SCALE_WAIT_S after the wake (same cap) for a scaled one. Still none -> refused. [guard].
GUARD_URL = os.environ.get("EBO_DEPTH_GUARD_URL", "").strip()
GUARD_CONFIG = os.environ.get("EBO_GUARD_CONFIG", "").strip()
GUARD_START_WAIT_S = float(os.environ.get("EBO_GUARD_START_WAIT_S", "5"))
GUARD_WAKE_WAIT_S = float(os.environ.get("EBO_GUARD_WAKE_WAIT_S", "3"))
GUARD_WAKE_MAX_S = float(os.environ.get("EBO_GUARD_WAKE_MAX_S", "10"))
GUARD_SCALE_WAIT_S = float(os.environ.get("EBO_GUARD_SCALE_WAIT_S", "8"))
GUARD_VIDEO_KEEP_URL = os.environ.get("EBO_GUARD_VIDEO_KEEP_URL", "").strip()
GUARD_DRIVE_WAIT_S = 0.3   # /drive: how long one call waits for a verdict before deciding to wake
GUARD_STALE_S = 0.5
GUARD_POLL_S = 0.03
STOP_FREE_DEFAULT, STOP_FREE_MIN, STOP_FREE_MAX = 0.25, 0.05, 1.5


class DepthGuard:
    def __init__(self, url, cfg_path, user, password):
        self.url, self.cfg_path = url, cfg_path
        self.hdr = ({"Authorization": "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()}
                    if user else {})
        self.op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.lock = threading.Lock()   # motor frames of guarded moves and guard stops go through it
        self.poll_until = 0.0          # monotonic: poll the depth until then
        self.forward_until = 0.0       # monotonic: forward motion commanded until then
        self.move_id = 0
        self.trip = None               # {"move_id", "reason", reading...} of the last guard stop
        self.verdict = (False, {"reason": "no depth reading yet"}, 0.0)   # (ok, info, monotonic t)
        self._cfg = (None, STOP_FREE_DEFAULT)
        self._wake_lock = threading.Lock()
        self._waking = False
        self.drive_wait = threading.Lock()   # /drive: only one call at a time waits for a wake
        threading.Thread(target=self._loop, name="depth-guard", daemon=True).start()

    def stop_free_m(self):
        try:
            mt = os.stat(self.cfg_path).st_mtime if self.cfg_path else None
        except OSError:
            mt = None
        if mt is None:
            return STOP_FREE_DEFAULT
        if mt != self._cfg[0]:
            v = STOP_FREE_DEFAULT
            try:
                with open(self.cfg_path) as f:
                    x = json.load(f).get("stop_free_m")
                if isinstance(x, (int, float)) and not isinstance(x, bool) and STOP_FREE_MIN <= x <= STOP_FREE_MAX:
                    v = float(x)
            except (OSError, ValueError, AttributeError):
                pass
            self._cfg = (mt, v)
        return self._cfg[1]

    def check(self):
        """Poll the depth once -> (ok, info). Anything but a fresh reading with enough room is not ok."""
        stop = self.stop_free_m()
        info = {"stop_free_m": stop}
        try:
            with self.op.open(urllib.request.Request(self.url, headers=self.hdr), timeout=0.3) as r:
                st = json.loads(r.read())
        except Exception as e:
            info["reason"] = f"depth down ({type(e).__name__})"
            return False, info
        info["depth_running"] = bool(st.get("running"))
        rd = st.get("reading")
        if not rd:
            info["reason"] = "no depth reading yet" if st.get("running") else "depth not running"
            info["no_fresh_reading"] = True
            return False, info
        free = rd.get("free_ahead_m")
        age = time.time() - float(rd.get("frame_t") or 0)
        info.update(free_ahead_m=free, free_ahead_kind=rd.get("free_ahead_kind"), depth_age_s=round(age, 2),
                    floor=rd.get("floor_source"))
        if age > GUARD_STALE_S:
            info["reason"] = f"depth stale ({age:.2f} s old)"
            info["no_fresh_reading"] = True
        elif free is None:
            info["reason"] = "distance ahead unknown (no floor scale)"
            info["no_scale"] = True
        elif rd.get("view_blocked"):
            info["reason"] = "view blocked"
        elif free < stop:
            info["reason"] = f"too close: {free:.2f} m free ahead < stop_free_m {stop:.2f}"
        else:
            return True, info
        return False, info

    def _loop(self):
        while True:
            now = time.monotonic()
            if now >= self.poll_until and now >= self.forward_until:
                time.sleep(0.01)
                continue
            ok, info = self.check()
            self.verdict = (ok, info, time.monotonic())
            if not ok:
                with self.lock:
                    if time.monotonic() < self.forward_until:
                        self.forward_until = 0.0
                        self.trip = {"move_id": self.move_id, **info}
                        for _ in range(3):
                            bridge.send_rdt(motor_frame())
                if self.trip and self.trip.get("move_id") == self.move_id and "logged" not in self.trip:
                    self.trip["logged"] = True
                    print(f"[guard] STOP: {json.dumps(info)}", flush=True)
            time.sleep(GUARD_POLL_S)

    def want(self, s=1.0):
        self.poll_until = max(self.poll_until, time.monotonic() + s)

    def fresh_verdict(self, since, max_age=0.15):
        ok, info, t = self.verdict
        return (ok, info) if t >= since and time.monotonic() - t <= max_age else None

    def _video_keep(self):
        req = urllib.request.Request(GUARD_VIDEO_KEEP_URL, data=b"{}", method="POST",
                                     headers={**self.hdr, "Content-Type": "application/json"})
        with self.op.open(req, timeout=15) as r:
            r.read()

    def wake(self):
        """No fresh depth reading: make sure the video stream runs (the depth decoder is kept running
        by the guard's /depth/live polls). Like ebo-eye: its POST /video_keep (also resets its idle
        pause timer), then - if video is still not publishing - the bridge's own video start.
        Runs in the background; returns at once (False = a wake is already in progress)."""
        with self._wake_lock:
            if self._waking:
                return False
            self._waking = True

        def run():
            how, t0 = [], time.monotonic()
            try:
                if GUARD_VIDEO_KEEP_URL:
                    try:
                        self._video_keep()
                        how.append("eye video_keep")
                    except Exception as e:
                        how.append(f"eye video_keep failed ({type(e).__name__})")
                if bridge.paused:
                    how.append("bridge paused (connection released), video not started")
                elif not bridge.video_ready():
                    how.append("bridge video start " + ("ok" if bridge.set_video(True) else "timed out"))
                else:
                    how.append("video publishing")
            except Exception as e:
                how.append(f"failed ({type(e).__name__})")
            finally:
                with self._wake_lock:
                    self._waking = False
            print(f"[guard] wake: {'; '.join(how)} ({time.monotonic() - t0:.2f} s)", flush=True)

        threading.Thread(target=run, name="guard-wake", daemon=True).start()
        return True

    def stream_up(self, info):
        """Video publishing and the depth decoder running (a fresh reading should follow)."""
        return bool(info.get("depth_running")) and bridge.video_ready()

    def wait_ready(self, timeout, wake_wait=None, scale_wait=None):
        """Before a forward move: one fresh verdict taken after this call; wait for a good one.
        If the verdict says there is no fresh reading, wake the stream (see wake) and wait until
        it is up plus wake_wait (GUARD_WAKE_WAIT_S) for a fresh reading, at most GUARD_WAKE_MAX_S
        after the wake; never shorter than timeout. After a wake only a fresh reading with a floor
        scale (free_ahead_m not null) decides (ok or refused at once); fresh unscaled readings extend
        the wait to scale_wait (GUARD_SCALE_WAIT_S) after the wake, same cap. The result's info
        carries "woke" then."""
        wake_wait = GUARD_WAKE_WAIT_S if wake_wait is None else wake_wait
        scale_wait = GUARD_SCALE_WAIT_S if scale_wait is None else scale_wait
        t0 = time.monotonic()
        deadline = t0 + timeout
        woke = up_since = None
        fresh_seen = False   # after the wake: a fresh reading arrived (maybe without a floor scale)
        last = (False, {"reason": "no depth reading yet", "stop_free_m": self.stop_free_m()})
        while True:
            now = time.monotonic()
            self.want()
            v = self.fresh_verdict(t0)
            if v:
                last = v
                if v[0]:
                    break
                if woke is None and v[1].get("no_fresh_reading"):
                    woke = now
                    print(f"[guard] no fresh depth reading ({v[1].get('reason')}): waking the stream, "
                          f"waiting up to {wake_wait:.1f} s once it is up", flush=True)
                    self.wake()
                if woke is not None:
                    if v[1].get("no_scale"):
                        if not fresh_seen:
                            fresh_seen = True
                            print(f"[guard] fresh reading without a floor scale after the wake: waiting "
                                  f"up to {scale_wait:.1f} s after the wake for a scaled one", flush=True)
                    elif not v[1].get("no_fresh_reading") and v[1].get("free_ahead_m") is not None:
                        break   # fresh and scaled but not ok (too close / blocked): decide now - refused
                    if self.stream_up(v[1]):
                        up_since = up_since or now
                    else:
                        up_since = None
                    end = (up_since + wake_wait) if up_since else woke + GUARD_WAKE_MAX_S
                    if fresh_seen:
                        end = max(end, woke + scale_wait)
                    end = min(woke + GUARD_WAKE_MAX_S, end)
                    deadline = max(t0 + timeout, end)
            if now >= deadline:
                break
            time.sleep(0.02)
        if woke is not None:
            ok, info = last
            info = {**info, "woke": True, "waited_s": round(time.monotonic() - t0, 2)}
            print(f"[guard] after wake: {'fresh reading' if ok else 'still refused'} "
                  f"{json.dumps(info)}", flush=True)
            last = (ok, info)
        return last

    def wait_reading(self, max_s):
        """Before the firmware dock routine (CMD_DOCK, not a bridge move): make sure depth readings
        flow - wake the stream if there is no fresh reading (the polls also hold the depth decoder) -
        and wait up to max_s for a fresh reading WITH a floor scale (free_ahead_m not null). Never
        refuses (the robot must still be able to dock): returns what it got."""
        t0 = time.monotonic()
        woke, got = False, None
        while True:
            now = time.monotonic()
            self.want()
            v = self.fresh_verdict(t0)
            if v:
                info = v[1]
                if info.get("no_fresh_reading"):
                    if not woke:
                        woke = True
                        print(f"[dock] no fresh depth reading ({info.get('reason')}): waking the stream, "
                              f"waiting up to {max_s:.1f} s before the dock routine", flush=True)
                        self.wake()
                else:
                    got = info
                    if info.get("free_ahead_m") is not None:
                        break
            if now - t0 >= max_s:
                break
            time.sleep(0.02)
        out = {"waited_s": round(time.monotonic() - t0, 2), "woke": woke, "reading": got is not None,
               "scaled": bool(got and got.get("free_ahead_m") is not None),
               "free_ahead_m": got.get("free_ahead_m") if got else None}
        print(f"[dock] prewake {json.dumps(out)}", flush=True)
        return out


guard = None


# ---- dock guard rail: while on the charging dock only a straight forward move (drives it off) ----
DOCK_REASON = "on the dock: only straight forward allowed — drive off first"
_dock_log_last = {}


def dock_refusal(ly: float, rx: float, client: str = "-", via: str = "move"):
    """None = allowed. While the robot is charging/docked (charge == 1) or its charge state is
    unknown/odd (anything but 0 = unplugged), only a straight FORWARD move (ly > 0 and rx == 0) is
    allowed; any turn or backwards move is refused. A pure stop (ly == 0 and rx == 0) is always fine.
    Undock/dock actions are not moves and are unaffected. No motor frame is sent on refusal."""
    charge = bridge.charge if bridge else -1
    if charge == 0 or (ly == 0 and rx == 0) or (ly > 0 and rx == 0):
        return None
    why = DOCK_REASON if charge == 1 else DOCK_REASON.replace("on the dock", "charge state unknown, assuming on the dock")
    info = {"ok": False, "refused": True, "refused_by": "dock", "dock_guard": True,
            "reason": why, "charge": charge, "ly": ly, "rx": rx}
    key = (via, charge, ly > 0, ly < 0, rx != 0)
    now = time.monotonic()
    if now - _dock_log_last.get(key, -1e9) >= 1.0:   # /drive posts at ~15 Hz: log once a second per kind
        _dock_log_last[key] = now
        print(f"[dock] REFUSED {via} {json.dumps({**info, 'client': client})}", flush=True)
    return info


def _guard_result(info, phase, moved_s=0.0):
    out = {"ok": False, "stopped_by_guard": True, "phase": phase, "moved_s": round(moved_s, 2)}
    out.update({k: v for k, v in info.items() if k not in ("move_id", "logged")})
    return out


def do_move(ly: float, rx: float, duration: float = 0.4):
    """Returns {"ok": True, ...} or, when the depth guard stopped/refused a forward move,
    {"ok": False, "stopped_by_guard": True, "phase": "start"|"moving", "reason", "free_ahead_m", ...}."""
    n = max(1, int(duration * 20))
    if guard is None or ly <= 0:
        for _ in range(n):
            bridge.send_rdt(motor_frame(ly=ly, rx=rx)); time.sleep(0.05)
        bridge.send_rdt(motor_frame())
        return {"ok": True} if guard is not None or ly <= 0 else {"ok": True, "guard": "off"}
    ok, info = guard.wait_ready(GUARD_START_WAIT_S)
    if not ok:
        bridge.send_rdt(motor_frame())
        print(f"[guard] refused forward move: {json.dumps(info)}", flush=True)
        return _guard_result(info, "start")
    with guard.lock:
        guard.move_id += 1
        mid = guard.move_id
        guard.trip = None
        guard.forward_until = time.monotonic() + duration + 0.3
    start_info = info
    t0 = time.monotonic()
    tripped = None
    for _ in range(n):
        with guard.lock:
            tr = guard.trip
            if tr is not None and tr.get("move_id") == mid:
                tripped = tr
                break
            bridge.send_rdt(motor_frame(ly=ly, rx=rx))
        time.sleep(0.05)
    with guard.lock:
        if guard.move_id == mid:
            guard.forward_until = 0.0
        bridge.send_rdt(motor_frame())
    moved = time.monotonic() - t0
    if tripped is not None:
        return _guard_result(tripped, "moving", moved)
    ok, last, _ = guard.verdict
    g = {"free_ahead_m_start": start_info.get("free_ahead_m"),
         "free_ahead_m_end": last.get("free_ahead_m"), "stop_free_m": last.get("stop_free_m")}
    if start_info.get("woke"):
        g.update(woke=True, waited_s=start_info.get("waited_s"))
    return {"ok": True, "moved_s": round(moved, 2), "guard": g}


# ---- undock: a real straight-forward move off the charging dock ----
# ROLA's dead zone swallows short/low pulses (the old 6 frames of ly 0.5 over 0.3 s barely moved
# it), so undock drives straight forward at EBO_UNDOCK_LY for EBO_UNDOCK_S (0.6 x 2.0 s ~ 0.28 m)
# through the same forward depth guard as /move (refused / stopped when free ahead < stop_free_m),
# then stops and polls the charge state up to EBO_UNDOCK_CHARGE_POLL_S for 1 -> 0.
UNDOCK_LY = float(os.environ.get("EBO_UNDOCK_LY", "0.6"))
UNDOCK_S = float(os.environ.get("EBO_UNDOCK_S", "2.0"))
UNDOCK_CHARGE_POLL_S = float(os.environ.get("EBO_UNDOCK_CHARGE_POLL_S", "2.0"))


def do_undock(client: str = "-"):
    charge_before = bridge.charge
    res = do_move(UNDOCK_LY, 0.0, UNDOCK_S)
    moved = res.get("phase") != "start"   # refused at the start = not moved at all
    t0 = time.monotonic()
    while moved and bridge.charge != 0 and time.monotonic() - t0 < UNDOCK_CHARGE_POLL_S:
        time.sleep(0.1)
    charge_after = bridge.charge
    out = {**res, "action": "undock", "ly": UNDOCK_LY, "duration_s": UNDOCK_S,
           "charge_before": charge_before, "charge_after": charge_after,
           "left_dock": charge_before == 1 and charge_after == 0}
    if moved:
        out["charge_polled_s"] = round(time.monotonic() - t0, 2)
    print(f"[undock] {json.dumps({**out, 'client': client})}", flush=True)
    return out


# ---- dock watch: the firmware dock routine (CMD_DOCK) is not a bridge move, so the forward guard
# never sees it. The routine searches (turns), approaches the dock's axis FORWARD camera-first while
# weaving left-right, turns 180 deg and reverses in. After a dock action a DockWatch thread polls the
# depth (EBO_DOCKWATCH_URL, default the guard's /depth/live; each poll also holds its decoder) every
# EBO_DOCKWATCH_POLL_S until charge == 1 or EBO_DOCKWATCH_TIMEOUT_S, and STOPS the robot (motor-zero
# frames - whether they interrupt the firmware routine is untested) when the free distance in the
# WIDE corridor (depth reading free_ahead_wide_m: robot half width + depth config
# wide_corridor_margin_m per side; falls back to free_ahead_m) is < EBO_DOCKWATCH_STOP_M AND it has
# been shrinking over the last EBO_DOCKWATCH_SHRINK_N fresh readings (each step not growing, each
# step at most EBO_DOCKWATCH_MAX_STEP_M - bigger jumps are the view sweeping in a turn - and at least
# EBO_DOCKWATCH_MIN_SHRINK_M in total) = driving toward it. Reversing into the dock (distance grows)
# and turning in place (distance jumps or holds) do not trigger. After a STOP it watches
# EBO_DOCKWATCH_CONFIRM_S more and re-sends the stop if the distance keeps closing
# (still_closing_after_stop). No reading / stale: it never stops (fail-open, "blind" counted) and
# wakes the stream. Logged [dockwatch]; GET /dockwatch, /info "dockwatch"; a STOP notice is attached
# once to the next /action or /move result ("dockwatch_notice"). EBO_DOCKWATCH=0 disables it.
DOCKWATCH_ON = os.environ.get("EBO_DOCKWATCH", "1").strip().lower() not in ("0", "false", "off", "no")
DOCKWATCH_URL = os.environ.get("EBO_DOCKWATCH_URL", "").strip()
DOCKWATCH_TIMEOUT_S = float(os.environ.get("EBO_DOCKWATCH_TIMEOUT_S", "120"))
DOCKWATCH_STOP_M = float(os.environ.get("EBO_DOCKWATCH_STOP_M", "0.25"))
DOCKWATCH_POLL_S = float(os.environ.get("EBO_DOCKWATCH_POLL_S", "0.1"))
DOCKWATCH_SHRINK_N = max(2, int(os.environ.get("EBO_DOCKWATCH_SHRINK_N", "3")))
DOCKWATCH_MIN_SHRINK_M = float(os.environ.get("EBO_DOCKWATCH_MIN_SHRINK_M", "0.02"))
DOCKWATCH_MAX_STEP_M = float(os.environ.get("EBO_DOCKWATCH_MAX_STEP_M", "0.15"))
DOCKWATCH_STALE_S = float(os.environ.get("EBO_DOCKWATCH_STALE_S", "1.0"))
DOCKWATCH_CONFIRM_S = float(os.environ.get("EBO_DOCKWATCH_CONFIRM_S", "3.0"))
DOCKWATCH_WAKE_EVERY_S = 5.0
# Recovery (Juraj 2026-10-02): after a STOP the obstacle is usually clearly on one side (the routine
# already half-sees the dock). From the depth sectors (/depth/live "sectors" [deg (neg = left), m,
# kind] + free_ahead_wide_lateral_m) pick the side AWAY from the nearest obstacle (see dock_free_side);
# still unclear -> don't move, report. Then turn 90 deg toward it (rx EBO_DOCKWATCH_TURN_RX for
# EBO_DOCKWATCH_TURN_S), drive EBO_DOCKWATCH_SIDESTEP_M forward through the normal forward depth guard
# (refused -> turn 180 deg and try the other side once; refused again ->
# abort), turn 90 deg back and relaunch the dock (pre-wake + a new watch). At most
# EBO_DOCKWATCH_RECOVER_MAX retries, then "docking failed". EBO_DOCKWATCH_RECOVER=0 = stop only.
DOCKWATCH_RECOVER = os.environ.get("EBO_DOCKWATCH_RECOVER", "1").strip().lower() not in ("0", "false", "off", "no")
DOCKWATCH_RECOVER_MAX = max(0, int(os.environ.get("EBO_DOCKWATCH_RECOVER_MAX", "2")))
DOCKWATCH_TURN_RX = float(os.environ.get("EBO_DOCKWATCH_TURN_RX", "0.6"))
DOCKWATCH_TURN_S = float(os.environ.get("EBO_DOCKWATCH_TURN_S", "1.0"))
DOCKWATCH_SIDESTEP_M = float(os.environ.get("EBO_DOCKWATCH_SIDESTEP_M", "0.20"))
DOCKWATCH_SIDESTEP_LY = float(os.environ.get("EBO_DOCKWATCH_SIDESTEP_LY", "0.6"))
DOCKWATCH_SPEED_MPS = float(os.environ.get("EBO_DOCKWATCH_SPEED_MPS", "0.14"))
DOCKWATCH_SIDE_MIN_DIFF_M = float(os.environ.get("EBO_DOCKWATCH_SIDE_MIN_DIFF_M", "0.15"))
# side = AWAY from where the nearest obstacle is: the lateral offset of the closest wide-corridor point
# (depth free_ahead_wide_lateral_m, + = right) when |offset| >= this; else the nearest sector per side
# over the full visible range (+/-42 deg), else the mean per side (2026-10-02: the old +/-25 deg window
# missed a box at L42 and turned into it).
DOCKWATCH_SIDE_LAT_MIN_M = float(os.environ.get("EBO_DOCKWATCH_SIDE_LAT_MIN_M", "0.03"))


def dockwatch_config():
    return {"timeout_s": DOCKWATCH_TIMEOUT_S, "stop_m": DOCKWATCH_STOP_M, "poll_s": DOCKWATCH_POLL_S,
            "shrink_n": DOCKWATCH_SHRINK_N, "min_shrink_m": DOCKWATCH_MIN_SHRINK_M,
            "max_step_m": DOCKWATCH_MAX_STEP_M, "stale_s": DOCKWATCH_STALE_S,
            "confirm_s": DOCKWATCH_CONFIRM_S,
            "recover": {"enabled": DOCKWATCH_RECOVER, "max": DOCKWATCH_RECOVER_MAX, "turn_rx": DOCKWATCH_TURN_RX,
                        "turn_s": DOCKWATCH_TURN_S, "sidestep_m": DOCKWATCH_SIDESTEP_M,
                        "sidestep_ly": DOCKWATCH_SIDESTEP_LY, "speed_mps": DOCKWATCH_SPEED_MPS,
                        "side_min_diff_m": DOCKWATCH_SIDE_MIN_DIFF_M, "side_lat_min_m": DOCKWATCH_SIDE_LAT_MIN_M}}


def dock_approaching(values):
    """values: the last free distances (oldest first). True = shrinking steadily (driving toward it)."""
    if len(values) < DOCKWATCH_SHRINK_N:
        return False
    v = values[-DOCKWATCH_SHRINK_N:]
    steps = [a - b for a, b in zip(v, v[1:])]   # positive = closer
    return (all(0 <= s <= DOCKWATCH_MAX_STEP_M for s in steps)
            and v[0] - v[-1] >= DOCKWATCH_MIN_SHRINK_M)


def dock_free_side(sectors, wide_lateral=None):
    """Pick the side to sidestep to: AWAY from where the nearest obstacle actually is.
    sectors [[deg, m, kind], ...] (deg < 0 = left), wide_lateral = lateral offset (m, + = right) of the
    closest wide-corridor obstacle point (depth free_ahead_wide_lateral_m) or None.
    Returns (side "left"|"right"|None, info incl. "obstacle_side")."""
    def side_vals(sign):
        out = []
        for sec in sectors or []:
            try:
                deg, m = float(sec[0]), sec[1]
            except (TypeError, ValueError, IndexError):
                continue
            if m is None or deg == 0 or (deg < 0) != (sign < 0):
                continue
            out.append(float(m))
        return out

    def away(obstacle, basis, extra):
        return ("left" if obstacle == "right" else "right"), {**extra, "obstacle_side": obstacle, "basis": basis}

    info = {}
    if wide_lateral is not None:
        try:
            lat = float(wide_lateral)
        except (TypeError, ValueError):
            lat = None
        if lat is not None:
            info["obstacle_lateral_m"] = round(lat, 3)
            if abs(lat) >= DOCKWATCH_SIDE_LAT_MIN_M:
                return away("right" if lat > 0 else "left", "wide_min", info)
    ls, rs = side_vals(-1), side_vals(1)
    info.update(nearest_left_m=min(ls) if ls else None, nearest_right_m=min(rs) if rs else None)
    if ls and rs and abs(min(ls) - min(rs)) >= DOCKWATCH_SIDE_MIN_DIFF_M:
        return away("left" if min(ls) < min(rs) else "right", "nearest", info)
    info.update(far_left_m=round(sum(ls) / len(ls), 2) if ls else None,
                far_right_m=round(sum(rs) / len(rs), 2) if rs else None)
    if ls and rs and abs(info["far_left_m"] - info["far_right_m"]) >= DOCKWATCH_SIDE_MIN_DIFF_M:
        return away("left" if info["far_left_m"] < info["far_right_m"] else "right", "far", info)
    return None, {**info, "basis": "ambiguous"}


class DockWatch:
    def __init__(self, url, user, password):
        self.url = url
        self.hdr = ({"Authorization": "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()}
                    if user else {})
        self.op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.lock = threading.Lock()
        self.watch_id = 0
        self.current = None      # state of the running watch (None = idle)
        self.last_event = None   # last watch end / STOP event
        self.notice = None       # STOP message for the next /action or /move result
        self.last_event_stop = None
        self.next_attempt = 0    # recovery attempt number for the next start() (0 = user dock)

    def fetch(self):
        """One /depth/live poll -> its reading dict (or None)."""
        with self.op.open(urllib.request.Request(self.url, headers=self.hdr), timeout=0.5) as r:
            return json.loads(r.read()).get("reading")

    def start(self, client="-", prewake=None):
        with self.lock:
            self.watch_id += 1
            wid = self.watch_id
            attempt, self.next_attempt = self.next_attempt, 0
            self.current = {"watch_id": wid, "started": time.time(), "client": client, "readings": 0,
                            "blind": 0, "unscaled": 0, "last_free_m": None, "corridor": None, "history": [],
                            "stopped": False, "prewake": prewake, "attempt": attempt}
        print(f"[dockwatch] watching dock routine #{wid} {json.dumps(dockwatch_config())}", flush=True)
        threading.Thread(target=self._run, args=(wid,), name="dockwatch", daemon=True).start()
        return {"watching": True, "watch_id": wid, "stop_m": DOCKWATCH_STOP_M, "timeout_s": DOCKWATCH_TIMEOUT_S}

    def pre_check(self):
        """Before CMD_DOCK: a fresh reading whose WIDE free distance is already < DOCKWATCH_STOP_M
        -> {"free_m", "corridor", "sectors", "kind"}; otherwise None (dock normally)."""
        try:
            rd = self.fetch()
        except Exception:
            return None
        if not rd or time.time() - float(rd.get("frame_t") or 0) > DOCKWATCH_STALE_S:
            return None
        if "free_ahead_wide_m" in rd:
            free, kind, corridor = rd.get("free_ahead_wide_m"), rd.get("free_ahead_wide_kind"), "wide"
            if rd.get("wide_half_width_m") is not None:
                corridor = f"wide +/-{rd['wide_half_width_m']:.2f} m"
        else:
            free, kind, corridor = rd.get("free_ahead_m"), rd.get("free_ahead_kind"), "narrow (no wide corridor)"
        if free is None or float(free) >= DOCKWATCH_STOP_M:
            return None
        return {"free_m": float(free), "kind": kind, "corridor": corridor, "sectors": rd.get("sectors"),
                "wide_lateral_m": rd.get("free_ahead_wide_lateral_m")}

    def start_blocked(self, prewake=None, pre=None, client="-"):
        """Blocked before docking: open a watch entry and run the sidestep recovery in the background
        (it relaunches the dock itself); counts as a recovery attempt."""
        with self.lock:
            self.watch_id += 1
            wid = self.watch_id
            attempt, self.next_attempt = self.next_attempt, 0
            self.current = {"watch_id": wid, "started": time.time(), "client": client, "readings": 1,
                            "blind": 0, "unscaled": 0, "last_free_m": pre["free_m"], "corridor": pre["corridor"],
                            "history": [pre["free_m"]], "stopped": True, "prewake": prewake,
                            "attempt": attempt, "pre_dock_blocked": True}
        ev = {"kind": "stopped", "pre_dock": True, "free_m": pre["free_m"], "corridor": pre["corridor"],
              "sectors": pre.get("sectors"), "wide_lateral_m": pre.get("wide_lateral_m"),
              "stop_m": DOCKWATCH_STOP_M,
              "message": f"blocked before docking: obstacle {pre['free_m']:.2f} m ahead"}
        print(f"[dockwatch] blocked before docking #{wid} (attempt {attempt}): {json.dumps(ev)}", flush=True)

        def run():
            if attempt >= DOCKWATCH_RECOVER_MAX:
                msg = (f"docking failed after {DOCKWATCH_RECOVER_MAX} attempts: blocked before docking, "
                       f"obstacle {pre['free_m']:.2f} m ahead")
                self._add_notice(msg)
                return self._end(wid, {**ev, "kind": "failed", "message": msg, "attempt": attempt})
            self._recover(wid, ev, attempt)
        threading.Thread(target=run, name="dockwatch-pre", daemon=True).start()
        return {"watching": True, "watch_id": wid, "pre_dock_blocked": True}

    def take_notice(self):
        with self.lock:
            n, self.notice = self.notice, None
        return n

    def _add_notice(self, msg):
        with self.lock:
            self.notice = msg if not self.notice else f"{self.notice} ; {msg}"

    def _attempt(self, wid):
        with self.lock:
            return (self.current or {}).get("attempt", 0) if self.watch_id == wid else 0

    def _recover(self, wid, stop_ev, attempt):
        """After a confirmed STOP (or blocked before docking): sidestep AWAY from the obstacle and relaunch
        the dock; if the sidestep forward is refused, turn 180 deg and try the other side once."""
        free = stop_ev.get("free_m")
        sectors, lateral = stop_ev.get("sectors"), stop_ev.get("wide_lateral_m")
        try:   # prefer a fresh reading (the robot has stopped)
            rd = self.fetch()
            if rd and rd.get("sectors") and time.time() - float(rd.get("frame_t") or 0) <= DOCKWATCH_STALE_S:
                sectors, lateral = rd["sectors"], rd.get("free_ahead_wide_lateral_m")
        except Exception:
            pass
        side, sinfo = dock_free_side(sectors, lateral)
        base = {**stop_ev, "side": side, "side_info": sinfo, "attempt": attempt + 1}
        pre_dock = bool(stop_ev.get("pre_dock"))
        what = "blocked before docking" if pre_dock else "docking blocked"
        if side is None:
            if pre_dock:
                msg = f"blocked before docking, side unclear: obstacle {free:.2f} m ahead - not moving"
            else:
                msg = f"docking blocked: obstacle {free:.2f} m ahead, no clearly freer side - not moving"
            self._add_notice(msg)
            return self._end(wid, {**base, "kind": "stopped", "recovery": "ambiguous", "message": msg})
        obstacle_side = sinfo.get("obstacle_side") or ("right" if side == "left" else "left")
        other = "right" if side == "left" else "left"
        rx = DOCKWATCH_TURN_RX if side == "right" else -DOCKWATCH_TURN_RX
        steps = []
        fwd_s = DOCKWATCH_SIDESTEP_M / max(DOCKWATCH_SPEED_MPS, 0.01)

        def gone():
            return self.watch_id != wid

        def docked():
            return bridge.charge == 1

        print(f"[dockwatch] recover {attempt + 1}/{DOCKWATCH_RECOVER_MAX}: obstacle {free:.2f} m on the "
              f"{obstacle_side}, sidestep {side} {json.dumps(sinfo)}", flush=True)
        r = do_move(0.0, rx, DOCKWATCH_TURN_S)
        steps.append({"turn": side, "ok": r.get("ok")})
        if gone():
            return
        if docked():
            return self._end(wid, {**base, "kind": "docked", "message": "docked (charging)", "steps": steps})
        r = do_move(DOCKWATCH_SIDESTEP_LY, 0.0, fwd_s)
        steps.append({"forward_m": DOCKWATCH_SIDESTEP_M, "side": side, "ok": r.get("ok"), "reason": r.get("reason")})
        went, back_rx = side, -rx
        if not r.get("ok"):
            first_reason = r.get("reason") or "guard"
            if gone():
                return
            # the chosen side is blocked too: turn 180 deg and try the other side once
            print(f"[dockwatch] sidestep {side} refused ({first_reason}) - turning 180 to try {other}", flush=True)
            r = do_move(0.0, -rx, 2 * DOCKWATCH_TURN_S)
            steps.append({"turn": other, "deg": 180, "ok": r.get("ok")})
            if gone():
                return
            if docked():
                return self._end(wid, {**base, "kind": "docked", "message": "docked (charging)", "steps": steps})
            r = do_move(DOCKWATCH_SIDESTEP_LY, 0.0, fwd_s)
            steps.append({"forward_m": DOCKWATCH_SIDESTEP_M, "side": other, "ok": r.get("ok"),
                          "reason": r.get("reason")})
            if not r.get("ok"):
                msg = (f"{what}: obstacle {free:.2f} m on the {obstacle_side} - sidestep {side} refused "
                       f"({first_reason}), sidestep {other} refused too ({r.get('reason') or 'guard'}); stopped")
                self._add_notice(msg)
                print(f"[dockwatch] recover aborted (both sides) {json.dumps(steps)}", flush=True)
                return self._end(wid, {**base, "kind": "recovery_aborted", "message": msg, "steps": steps})
            went, back_rx = other, rx
        if gone():
            return
        r = do_move(0.0, back_rx, DOCKWATCH_TURN_S)
        steps.append({"turn_back": "right" if back_rx > 0 else "left", "ok": r.get("ok")})
        if gone():
            return
        msg = (f"{what}: obstacle {free:.2f} m on the {obstacle_side} - sidestepped {went}"
               f"{'' if went == side else f' ({side} was blocked)'}, "
               f"{'docking' if pre_dock else 'retrying'} ({attempt + 1}/{DOCKWATCH_RECOVER_MAX})")
        self._add_notice(msg)
        self._end(wid, {**base, "kind": "recovering", "message": msg, "steps": steps})
        with self.lock:
            self.next_attempt = attempt + 1
        print(f"[dockwatch] relaunching dock (attempt {attempt + 1}/{DOCKWATCH_RECOVER_MAX})", flush=True)
        do_action("dock")

    def status(self):
        with self.lock:
            cur = dict(self.current) if self.current else None
            ev = dict(self.last_event) if self.last_event else None
        if cur:
            cur["elapsed_s"] = round(time.time() - cur["started"], 1)
        return {"enabled": True, "active": cur is not None, "watch": cur, "last_event": ev,
                "config": dockwatch_config()}

    def _stop_robot(self):
        lk = guard.lock if guard is not None else threading.Lock()
        with lk:
            for _ in range(5):
                bridge.send_rdt(motor_frame())
                time.sleep(0.02)

    def _end(self, wid, event):
        with self.lock:
            if self.watch_id != wid:
                return
            cur = self.current or {}
            event = {"watch_id": wid, "t": time.time(),
                     "elapsed_s": round(time.time() - cur.get("started", time.time()), 1),
                     "readings": cur.get("readings", 0), "blind": cur.get("blind", 0),
                     "unscaled": cur.get("unscaled", 0), "prewake": cur.get("prewake"), **event}
            self.last_event = event
            self.current = None
        print(f"[dockwatch] end {json.dumps(event)}", flush=True)

    def _run(self, wid):
        t0 = time.monotonic()
        values, last_seq, last_wake = [], None, -1e9
        stop_at = stop_free = None   # monotonic time / distance of the STOP
        while True:
            if self.watch_id != wid:
                return   # superseded by a newer dock action
            now = time.monotonic()
            if bridge.charge == 1:
                ev = {"kind": "docked", "message": "docked (charging)"}
                if stop_at is not None:
                    ev = {**self.last_event_stop, "kind": "stopped", "charging_after_stop": True}
                return self._end(wid, ev)
            if stop_at is not None and now - stop_at >= DOCKWATCH_CONFIRM_S:
                ev = {**self.last_event_stop, "final_free_m": values[-1] if values else None}
                if not DOCKWATCH_RECOVER:
                    return self._end(wid, ev)
                attempt = self._attempt(wid)
                if attempt >= DOCKWATCH_RECOVER_MAX:
                    msg = (f"docking failed after {DOCKWATCH_RECOVER_MAX} attempts: obstacle "
                           f"{ev.get('free_m', 0):.2f} m ahead")
                    self._add_notice(msg)
                    return self._end(wid, {**ev, "kind": "failed", "message": msg, "attempt": attempt})
                return self._recover(wid, ev, attempt)
            if now - t0 >= DOCKWATCH_TIMEOUT_S:
                return self._end(wid, {"kind": "timeout",
                                       "message": f"dock watch ended after {DOCKWATCH_TIMEOUT_S:.0f} s without charging"})
            try:
                rd = self.fetch()
            except Exception:
                rd = None
            fresh = (rd is not None and rd.get("seq") != last_seq
                     and time.time() - float(rd.get("frame_t") or 0) <= DOCKWATCH_STALE_S)
            if not fresh:
                if rd is None or time.time() - float(rd.get("frame_t") or 0) > DOCKWATCH_STALE_S:
                    with self.lock:
                        if self.current:
                            self.current["blind"] += 1
                    values = []
                    if guard is not None and now - last_wake >= DOCKWATCH_WAKE_EVERY_S:
                        last_wake = now
                        guard.wake()
                time.sleep(DOCKWATCH_POLL_S)
                continue
            last_seq = rd.get("seq")
            if "free_ahead_wide_m" in rd:
                free, kind, corridor = rd.get("free_ahead_wide_m"), rd.get("free_ahead_wide_kind"), "wide"
                if rd.get("wide_half_width_m") is not None:
                    corridor = f"wide +/-{rd['wide_half_width_m']:.2f} m"
            else:
                free, kind, corridor = rd.get("free_ahead_m"), rd.get("free_ahead_kind"), "narrow (no wide corridor)"
            if free is None or kind == "blocked":
                values = []   # unknown or blocked view: no trend
            else:
                values = (values + [float(free)])[-max(DOCKWATCH_SHRINK_N, 2):]
            with self.lock:
                if self.current and self.watch_id == wid:
                    self.current.update(readings=self.current["readings"] + 1, last_free_m=free,
                                        last_kind=kind, corridor=corridor, history=list(values),
                                        unscaled=self.current.get("unscaled", 0) + (free is None))
            if free is None or kind == "blocked":
                time.sleep(DOCKWATCH_POLL_S)
                continue
            if stop_at is None:
                if free < DOCKWATCH_STOP_M and dock_approaching(values):
                    self._stop_robot()
                    stop_at, stop_free = time.monotonic(), free
                    msg = f"docking stopped: obstacle {free:.2f} m ahead"
                    ev = {"kind": "stopped", "message": msg, "free_m": free, "corridor": corridor,
                          "history": list(values), "stop_m": DOCKWATCH_STOP_M,
                          "elapsed_s": round(now - t0, 1), "resends": 0, "sectors": rd.get("sectors"),
                          "wide_lateral_m": rd.get("free_ahead_wide_lateral_m")}
                    self.last_event_stop = ev
                    with self.lock:
                        if self.watch_id == wid:
                            if not DOCKWATCH_RECOVER:
                                self.notice = msg
                            self.last_event = {**ev, "watch_id": wid, "t": time.time(), "confirming": True}
                            if self.current:
                                self.current["stopped"] = True
                    print(f"[dockwatch] STOP {json.dumps(ev)}", flush=True)
            elif free <= stop_free - DOCKWATCH_MIN_SHRINK_M and dock_approaching(values):
                # still closing after the STOP: the firmware routine may ignore motor-zero frames
                self._stop_robot()
                stop_free = free
                self.last_event_stop = {**self.last_event_stop, "still_closing_after_stop": True,
                                        "resends": self.last_event_stop["resends"] + 1, "min_free_m": free}
                print(f"[dockwatch] still closing after STOP, stop re-sent {json.dumps(self.last_event_stop)}",
                      flush=True)
            time.sleep(DOCKWATCH_POLL_S)


dockwatch = None


# ---------------- HTTP API + web panel (optional basic auth) ----------------
bridge = None
_talk_owner = threading.Lock()

def _valid_basic_auth(header: str | None) -> bool:
    if not WEB_USER:
        return True
    try:
        scheme, encoded = (header or "").split(" ", 1)
        if scheme.lower() != "basic":
            return False
        decoded = base64.b64decode(encoded, validate=True).decode("utf-8")
        username, password = decoded.split(":", 1)
    except (ValueError, UnicodeError):
        return False
    return (secrets.compare_digest(username, WEB_USER) and
            secrets.compare_digest(password, WEB_PASS or ""))

def require_auth(connection: HTTPConnection):
    if connection.scope["type"] == "websocket":
        return  # WebSockets are checked explicitly before accept().
    if connection.url.path == "/sleep.svg":   # public: harmless fallback graphic, usable as a card image
        return
    if not _valid_basic_auth(connection.headers.get("authorization")):
        raise HTTPException(status_code=401, detail="Unauthorized", headers={"WWW-Authenticate": "Basic"})

app = FastAPI(dependencies=[Depends(require_auth)])

@app.get("/", response_class=HTMLResponse)
def index():
    p = Path(__file__).parent / "ebo.html"
    return p.read_text(encoding="utf-8") if p.exists() else "<h1>EBO-SE</h1>"

@app.get("/sleep.svg")
def sleep_image():
    p = Path(__file__).parent / "sleep.svg"
    if not p.exists():
        raise HTTPException(status_code=404)
    return Response(content=p.read_text(encoding="utf-8"), media_type="image/svg+xml")

# WHEP (WebRTC) signaling proxy: the panel (already behind web auth) posts its SDP offer
# here same-origin, and we forward it to mediamtx with the stream credentials in the
# Authorization header — so no credentials ever appear in a browser URL (Chrome blocks
# subresource URLs with embedded credentials). Media still flows peer-to-peer.
_WHEP_UP = f"http://127.0.0.1:8889/{RTSP_PATH}/whep"

def _whep_proxy(offer: bytes) -> Response:
    headers = {"Content-Type": "application/sdp"}
    if STREAM_PASS:
        tok = base64.b64encode(f"{STREAM_USER}:{STREAM_PASS}".encode()).decode()
        headers["Authorization"] = "Basic " + tok
    req = urllib.request.Request(_WHEP_UP, data=offer, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return Response(content=r.read(), media_type=r.headers.get("Content-Type", "application/sdp"))
    except urllib.error.HTTPError as e:
        return Response(content=e.read(), status_code=e.code, media_type="application/sdp")

@app.post("/whep")
async def whep(request: Request):
    offer = await request.body()
    return await run_in_threadpool(_whep_proxy, offer)

# HLS (LL-HLS) reverse proxy. WebRTC media is peer-to-peer and can't cross the Cloudflare tunnel,
# so when the panel is opened remotely it falls back to HLS, which is plain HTTP and proxied here
# (same-origin, already behind web auth; mediamtx's stream credentials are injected server-side).
def _hls_open(path: str, query: str):
    url = f"http://127.0.0.1:8888/{path}" + (("?" + query) if query else "")
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    req = urllib.request.Request(url)
    if STREAM_PASS:
        req.add_header("Authorization", "Basic " + base64.b64encode(f"{STREAM_USER}:{STREAM_PASS}".encode()).decode())
    return opener.open(req, timeout=20)   # response object (raises HTTPError on 4xx/5xx)

@app.get("/hls/{path:path}")
async def hls(path: str, request: Request):
    try:
        r = await run_in_threadpool(_hls_open, path, request.url.query)
    except urllib.error.HTTPError as e:
        return Response(content=e.read(), status_code=e.code,
                        media_type=e.headers.get("Content-Type", "application/json"))
    except Exception:
        return Response(content=b"", status_code=502)
    # Stream the bytes through (chunked) instead of buffering the whole response: lets LL-HLS
    # parts/segments flow promptly and avoids adding a full-response delay through the tunnel.
    def gen():
        try:
            while True:
                chunk = r.read(65536)
                if not chunk: break
                yield chunk
        finally:
            r.close()
    return StreamingResponse(gen(), media_type=r.headers.get("Content-Type", "application/octet-stream"))

@app.get("/info")
def info():
    ip = lan_ip()
    now = time.time()
    return JSONResponse({
        "connected": bridge.connected if bridge else False,
        "paused": bridge.paused if bridge else False,
        "awake": bridge.is_awake() if bridge else False,
        "codec": bridge.codec_name if bridge else None,
        "frames_received": bridge.frame_count if bridge else 0,
        "video_enabled": bridge.video_enabled if bridge else False,
        "video_ready": bridge.video_ready() if bridge else False,
        "video_quality": bridge.video_quality if bridge else None,
        "audio": ({"codec": f"0x{bridge.audio_codec:02x}", "flags": f"0x{bridge.audio_flags:02x}",
                   "count": bridge.audio_count, "active": now - bridge.last_audio < 2.0,
                   "last_received": bridge.last_audio}
                  if bridge and bridge.audio_codec is not None else None),
        "battery": bridge.battery if bridge else -1,
        "charge": bridge.charge if bridge else -1,
        "parameters": bridge.parameters if bridge else {},
        "telemetry": bridge.telemetry if bridge else {},
        "rtsp": f"rtsp://{ip}:8554/{RTSP_PATH}",
        "rtsp_audio": f"rtsp://{ip}:8554/{RTSP_AUDIO_PATH}",
        "dockwatch": dockwatch_brief(),
    })


def dockwatch_brief():
    """Short dock-watch state for /info (full state: GET /dockwatch)."""
    if dockwatch is None:
        return {"enabled": False}
    st = dockwatch.status()
    w, ev = st["watch"], st["last_event"]
    return {"enabled": True, "active": st["active"],
            "last_free_m": w.get("last_free_m") if w else None,
            "last_event": ({k: ev.get(k) for k in ("kind", "message", "free_m", "t", "still_closing_after_stop")
                            if k in ev} if ev else None)}


def with_dockwatch_notice(result):
    """Attach a pending dock-watch STOP notice (once) to an action/move result for the brother."""
    n = dockwatch.take_notice() if dockwatch is not None else None
    if n and isinstance(result, dict):
        result = {**result, "dockwatch_notice": n}
    return result


@app.get("/dockwatch")
def dockwatch_state():
    if dockwatch is None:
        return {"enabled": False, "reason": "EBO_DOCKWATCH=0 or no depth URL",
                "config": dockwatch_config()}
    return dockwatch.status()


class ParameterWrite(BaseModel):
    value: float


class VideoQualityWrite(BaseModel):
    level: str


@app.get("/parameters")
def parameters():
    return {"values": dict(bridge.parameters), "metadata": dict(bridge.parameter_meta),
            "supported": PARAMETERS}


@app.get("/parameter/{group}/{key}")
def parameter(group: str, key: str):
    name = f"{group}-{key}"
    if name not in PARAMETERS:
        raise HTTPException(status_code=404, detail="unsupported EBO SE parameter")
    value = bridge.read_parameter(group, key)
    if value is None:
        raise HTTPException(status_code=504, detail="robot did not return the parameter")
    return {"name": name, "value": value, "metadata": bridge.parameter_meta.get(name, {})}


@app.put("/parameter/{group}/{key}")
def parameter_write(group: str, key: str, body: ParameterWrite):
    try:
        result = bridge.set_parameter(group, key, body.value)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    if not result["acknowledged"]:
        raise HTTPException(status_code=504, detail={"message": "robot did not echo requested value", **result})
    return {"ok": True, "name": f"{group}-{key}", **result}


@app.get("/telemetry")
def telemetry():
    out = dict(bridge.telemetry)
    att = out.get("attitude")
    if att:   # firmware reports radians; add degrees for people and brothers
        out["attitude_deg"] = {k: round(math.degrees(att[k]), 2) for k in ("roll", "pitch", "yaw")}
    upd = getattr(bridge, "telemetry_updated", {})
    if upd:
        out["updated"] = {k: round(v, 1) for k, v in upd.items()}
    return out


@app.get("/telemetry/raw_ids")
def telemetry_raw_ids():
    """Diagnostics: which MAVLink message ids arrived over RDT, how often, and when."""
    now = time.time()
    return {str(k): {**v, "age_s": round(now - v["last"], 1)} for k, v in sorted(bridge.raw_ids.items())}


@app.get("/day-night")
def day_night():
    value = bridge.read_parameter("video", "night_vision")
    if value is None:
        raise HTTPException(status_code=504, detail="robot did not return day/night mode")
    mode = {0: "auto", 1: "day", 2: "night"}.get(int(value), "unknown")
    return {"value": value, "mode": mode, "colour_forced": int(value) == 1,
            "note": "auto is configured policy; firmware does not report the current IR-cut sensor state"}


@app.post("/video/{state}")
def video(state: str):
    if state not in ("start", "stop"):
        raise HTTPException(status_code=400, detail="state must be start or stop")
    ready = bridge.set_video(state == "start")
    if state == "start" and not ready:
        raise HTTPException(status_code=504, detail="video enabled but no publishable frame arrived before timeout")
    return {"ok": True, "video_enabled": bridge.video_enabled,
            "video_ready": bridge.video_ready(), "audio_preserved": True,
            "talkback_preserved": True}


@app.get("/video/quality")
def video_quality():
    return {
        "level": bridge.video_quality,
        "updated": bridge.video_quality_updated or None,
        "ioctrl": hex(VIDEO_QUALITY_IOCTL),
        "profiles": {
            name: {"target_resolution": profile[2],
                   "target_bitrate_kbps": profile[1] or None}
            for name, profile in VIDEO_QUALITY_PROFILES.items()
        },
        "note": "level is the last bridge request; this firmware provides no quality readback",
    }


@app.put("/video/quality")
def video_quality_write(body: VideoQualityWrite):
    try:
        result = bridge.set_video_quality(body.level)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, **result}


@app.post("/debug/ioctl")
def debug_ioctl(body: dict, request: Request):
    """Send one raw IOCTRL to the robot (local callers only). For probing
    undocumented commands, e.g. {"type": "0x320", "payload": "0000000002000000"}."""
    if request.client.host not in ("127.0.0.1", "::1"):
        raise HTTPException(status_code=403, detail="local only")
    io = int(str(body.get("type")), 0)
    data = bytes.fromhex(str(body.get("payload", "")))
    if not 0 < io < 0x10000 or len(data) > 256:
        raise HTTPException(status_code=400, detail="bad type or payload")
    bridge.send_ioctl(io, data)
    return {"ok": True, "type": hex(io), "bytes": len(data)}


@app.get("/video/ready")
def video_ready():
    return {"video_enabled": bridge.video_enabled, "video_ready": bridge.video_ready(),
            "frames_received": bridge.frame_count,
            "frames_published": bridge._published_frame_count}


@app.post("/photo", status_code=202)
def photo():
    # ROLA takes its UI screenshot and then asks the IPC to save the original
    # 1920x1080 frame to /DICM on the robot's SD card with this IOCTRL.
    bridge.send_ioctl(0x9901, bytes(8))
    return {"ok": True, "accepted": True, "storage": "/DICM"}


def _media_list():
    with bridge._media_lock:
        client = None
        try:
            client = bridge.open_media_client()
            return client.list_media()
        finally:
            if client:
                client.close()
            try: bridge.send_rdt(MAVLINK.file_server_close())
            except Exception: pass


@app.get("/media")
def media():
    try:
        items = _media_list()
    except (TimeoutError, OSError, MediaProtocolError) as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {"count": len(items), "items": items}


@app.get("/media/{name}")
def media_file(name: str):
    if not name or name != Path(name).name or "/" in name or "\\" in name:
        raise HTTPException(status_code=400, detail="name must be a media basename")
    tmp_path = None
    with bridge._media_lock:
        client = None
        try:
            client = bridge.open_media_client()
            matches = [item for item in client.list_media() if item["name"] == name]
            if not matches:
                raise HTTPException(status_code=404, detail="media not found")
            if len(matches) > 1:
                raise HTTPException(status_code=409, detail="media basename is ambiguous")
            item = matches[0]
            maximum = int(os.environ.get("EBO_MEDIA_MAX_BYTES", str(512 * 1024 * 1024)))
            if item["size"] > maximum:
                raise HTTPException(status_code=413, detail="media exceeds EBO_MEDIA_MAX_BYTES")
            fd, raw_path = tempfile.mkstemp(prefix="ebo-media-", suffix=Path(name).suffix)
            os.close(fd)
            tmp_path = Path(raw_path)
            received = client.download(item["path"], tmp_path, item["size"])
            if received != item["size"]:
                raise MediaProtocolError(f"short media download ({received}/{item['size']})")
        except HTTPException:
            if tmp_path: tmp_path.unlink(missing_ok=True)
            raise
        except (TimeoutError, OSError, MediaProtocolError) as exc:
            if tmp_path: tmp_path.unlink(missing_ok=True)
            raise HTTPException(status_code=502, detail=str(exc)) from exc
        finally:
            if client:
                client.close()
            try: bridge.send_rdt(MAVLINK.file_server_close())
            except Exception: pass
    media_type = "image/jpeg" if name.lower().endswith(".jpg") else "video/x-msvideo"
    return FileResponse(tmp_path, filename=name, media_type=media_type,
                        background=BackgroundTask(tmp_path.unlink, missing_ok=True))


@app.get("/auto-return")
def auto_return():
    tasks = bridge.request_timed_tasks(3)
    full_day_dnd = any(task["enabled"] and task["weekdays"] == 0x7f and
                       task["start_hour"] == 0 and task["start_minute"] == 0 and
                       task["end_hour"] == 23 and task["end_minute"] == 59
                       for task in tasks)
    return {"auto_return": not full_day_dnd, "do_not_disturb_tasks": tasks,
            "derived": True}


class ScheduledSecurityTask(BaseModel):
    task_id: int = 0
    task_type: int
    enabled: bool = True
    weekdays: int = 0x7F
    start_hour: int
    start_minute: int
    end_hour: int
    end_minute: int
    security_type: int = 0
    security_disabled: int = 0
    security_air_mode: int = 0


@app.get("/security/tasks/{task_type}")
def security_tasks(task_type: int):
    try:
        tasks = bridge.request_timed_tasks(task_type)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"task_type": task_type, "tasks": tasks}


@app.put("/security/tasks")
def security_task_write(body: ScheduledSecurityTask):
    """Create/update a fixed-point (1), cruise (2), or do-not-disturb (3) schedule.

    Enabling a cruise entry authorizes later autonomous movement at its scheduled
    time.  The endpoint never starts an immediate patrol.
    """
    values = body.model_dump() if hasattr(body, "model_dump") else body.dict()
    task = TimedTask(**values)
    try:
        task.validate()
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    ack = bridge.set_timed_task(task)
    accepted = bool(ack and ack.get("result") == 0)
    if not accepted:
        raise HTTPException(status_code=504, detail={"message": "robot did not accept scheduled task", "ack": ack})
    return {"ok": True, "ack": ack, "warning": "enabled cruise tasks can move the robot at the scheduled time"}

class Move(BaseModel):
    ly: float = Field(default=0.0, ge=-1.0, le=1.0)
    rx: float = Field(default=0.0, ge=-1.0, le=1.0)
    duration: float = Field(default=0.4, gt=0.0, le=2.0)

@app.post("/move")
def move(m: Move, request: Request):
    refusal = dock_refusal(m.ly, m.rx, request.client.host if request.client else "-", "move")
    if refusal is not None:
        return with_dockwatch_notice(refusal)
    return with_dockwatch_notice(do_move(m.ly, m.rx, m.duration))

# Analog joystick: the panel POSTs the current vector at ~15 Hz while dragging. Each call
# sends ONE motor frame (no trailing stop) so motion is continuous; a server-side watchdog
# stops the robot if updates stop arriving (release, navigation away, network drop).
_last_drive = 0.0
_stop_count = 0   # POST /stop calls (a /drive that waited for depth does not move after one)

@app.post("/drive")
def drive(m: Move, request: Request):
    global _last_drive
    refusal = dock_refusal(m.ly, m.rx, request.client.host if request.client else "-", "drive")
    if refusal is not None:
        return refusal
    if guard is not None and m.ly > 0:
        # forward: only on a fresh good depth verdict; the guard thread stops it mid-way
        t_call = time.monotonic()
        guard.want(1.0)
        v = guard.fresh_verdict(t_call - 0.15)
        if v is None or (not v[0] and v[1].get("no_fresh_reading")):
            # guard idle or no fresh reading (video paused): one call waits (waking the stream
            # if needed), the others are refused at once; nothing moves after a /stop meanwhile
            if not guard.drive_wait.acquire(blocking=False):
                bridge.send_rdt(motor_frame())
                return _guard_result({"reason": "waiting for depth (waking the stream)",
                                      "stop_free_m": guard.stop_free_m()}, "drive")
            try:
                stops = _stop_count
                v = guard.wait_ready(GUARD_DRIVE_WAIT_S)
            finally:
                guard.drive_wait.release()
            if _stop_count != stops:
                return _guard_result({**v[1], "reason": "stopped while waiting for depth"}, "drive")
            t_call = time.monotonic()
        with guard.lock:
            if v is None or v[0]:   # re-check atomically with the motor frame
                v = guard.fresh_verdict(t_call - 0.15)
            if v is None or not v[0]:
                bridge.send_rdt(motor_frame())
                _last_drive = 0.0
                info = v[1] if v else {"reason": "waiting for depth", "stop_free_m": guard.stop_free_m()}
                return _guard_result(info, "drive")
            guard.forward_until = time.monotonic() + 0.4
            bridge.send_rdt(motor_frame(ly=m.ly, rx=m.rx))
        _last_drive = time.time()
        return {"ok": True}
    bridge.send_rdt(motor_frame(ly=m.ly, rx=m.rx))
    _last_drive = time.time()
    return {"ok": True}

def _drive_watchdog():
    global _last_drive
    while True:
        time.sleep(0.1)
        if _last_drive and (time.time() - _last_drive) > 0.35:
            _last_drive = 0.0
            try: bridge.send_rdt(motor_frame())   # deadman stop
            except Exception: pass

@app.post("/stop")
def stop():
    global _last_drive, _stop_count
    _last_drive = 0.0
    _stop_count += 1
    for _ in range(3): bridge.send_rdt(motor_frame()); time.sleep(0.02)
    return {"ok": True}

BASIC_ACTIONS = ("dock", "undock", "autoreturn_off", "autoreturn_on")


def action_catalog():
    """Every name POST /action/{name} accepts; tricks (firmware motion routines) carry moves=True."""
    out = [{"name": n, "moves": False} for n in BASIC_ACTIONS + tuple(PARAM_TOGGLES)]
    out += [{"name": n, "moves": True, "trick": True, "command": c}
            for n, c in SPECIAL_MOTION_COMMANDS.items()]
    return out


@app.get("/actions")
def actions():
    return {"actions": action_catalog()}


def trick_refusal(force: bool):
    """Why a trick must not start now (None = allowed). Tricks are firmware routines that run on
    their own once sent (the bridge cannot meter or reliably stop them), so they are gated at the
    start: refused while the robot is charging/docked or its charge state is unknown, and - when the
    depth guard is on - unless one fresh depth reading shows room ahead. force=True skips both."""
    if force:
        return None
    charge = bridge.charge if bridge else -1
    if charge != 0:
        return {"reason": "robot is charging/docked" if charge == 1 else f"charge state not unplugged ({charge})",
                "charge": charge}
    if guard is not None:
        ok, info = guard.wait_ready(GUARD_START_WAIT_S)
        if not ok:
            return {**{k: v for k, v in info.items() if k != "reason"},
                    "reason": f"depth guard: {info.get('reason')}"}
    return None


def do_trick(name: str, force: bool = False, client: str = "-"):
    """Gate, log and send one trick. Every call is logged ([trick] SENT / REFUSED / FAILED)."""
    base = {"action": name, "command": SPECIAL_MOTION_COMMANDS[name], "force": force,
            "charge": bridge.charge if bridge else -1, "battery": bridge.battery if bridge else -1,
            "client": client}
    refusal = trick_refusal(force)
    if refusal is not None:
        print(f"[trick] REFUSED {json.dumps({**base, **refusal})}", flush=True)
        return {"ok": False, "refused": True, "action": name, **refusal}
    try:
        do_action(name)
    except Exception as exc:
        print(f"[trick] FAILED {json.dumps({**base, 'error': type(exc).__name__})}", flush=True)
        return {"ok": False, "action": name, "err": type(exc).__name__}
    print(f"[trick] SENT {json.dumps(base)}", flush=True)
    return {"ok": True, "action": name, "trick": True, "force": force}


@app.post("/action/{name}")
def action(name: str, request: Request, force: int = 0):
    client = request.client.host if request.client else "-"
    if name in SPECIAL_MOTION_COMMANDS:
        return with_dockwatch_notice(do_trick(name, bool(force), client))
    if name == "undock":
        return with_dockwatch_notice(do_undock(client))
    if name == "dock":
        notice = dockwatch.take_notice() if dockwatch is not None else None   # an older STOP
        do_action("dock")
        out = {"ok": True, "action": "dock"}
        if dockwatch is not None:
            w = dockwatch.status()["watch"] or {}
            out["dockwatch"] = {"watching": True, "watch_id": w.get("watch_id"), "stop_m": DOCKWATCH_STOP_M,
                                "timeout_s": DOCKWATCH_TIMEOUT_S}
            if w.get("pre_dock_blocked"):
                out["dockwatch"]["pre_dock_blocked"] = True
        else:
            out["dockwatch"] = {"watching": False}
        if _last_dock_prewake is not None:
            out["prewake"] = _last_dock_prewake
        if notice:
            out["dockwatch_notice"] = notice
        return out
    return with_dockwatch_notice({"ok": True, "action": name} if do_action(name) else {"ok": False, "err": "unknown"})

@app.post("/connection/{state}")
def connection(state: str):
    # state = "stop" (release the robot for the Enabot app) or "start" (reconnect)
    if state == "stop":
        bridge.pause_bridge()
    elif state == "start":
        bridge.resume_bridge()
    else:
        return {"ok": False, "err": "unknown"}
    return {"ok": True, "paused": bridge.paused}


@app.websocket("/ws/talk")
async def talk(websocket: WebSocket):
    if not _valid_basic_auth(websocket.headers.get("authorization")):
        await websocket.close(code=4401, reason="Unauthorized")
        return
    if not _talk_owner.acquire(blocking=False):
        await websocket.close(code=4409, reason="Talkback already in use")
        return

    started = False
    try:
        await websocket.accept()
        if not bridge or bridge.paused:
            await websocket.send_json({"type": "error", "error": "bridge unavailable"})
            await websocket.close(code=1013)
            return
        try:
            await run_in_threadpool(bridge.speaker_start, 12.0)
            started = True
        except Exception as exc:
            print("[supervisor] talkback start failed:", type(exc).__name__, flush=True)
            await websocket.send_json({"type": "error", "error": "speaker start failed"})
            await websocket.close(code=1011)
            return

        await websocket.send_json({"type": "ready", "format": "pcm_s16le", "rate": 8000,
                                   "channels": 1, "max_bytes": 1280})
        # At 8 kHz mono PCM16 the natural rate is 16 kB/s. Allow a 200 ms burst,
        # then reject clients that continuously outrun real time.
        budget = 3200.0
        last = time.monotonic()
        while True:
            pcm = await websocket.receive_bytes()
            now = time.monotonic()
            budget = min(3200.0, budget + (now - last) * 16000.0)
            last = now
            if not pcm or len(pcm) > 1280 or len(pcm) % 2 or len(pcm) > budget:
                await websocket.close(code=1008, reason="Invalid PCM stream")
                return
            budget -= len(pcm)
            await run_in_threadpool(bridge.speaker_send_pcm, pcm)
    except WebSocketDisconnect:
        pass
    finally:
        if started and bridge:
            try: await run_in_threadpool(bridge.speaker_stop)
            except Exception: pass
        _talk_owner.release()


if __name__ == "__main__":
    bridge = Bridge()
    threading.Thread(target=_drive_watchdog, daemon=True).start()
    if GUARD_URL:
        guard = DepthGuard(GUARD_URL, GUARD_CONFIG, WEB_USER, WEB_PASS)
        print(f"[guard] forward-move depth guard on (config {GUARD_CONFIG or '-'}, "
              f"stop_free_m {guard.stop_free_m():.2f})", flush=True)
    if DOCKWATCH_ON and (DOCKWATCH_URL or GUARD_URL):
        dockwatch = DockWatch(DOCKWATCH_URL or GUARD_URL, WEB_USER, WEB_PASS)
        print(f"[dockwatch] on ({DOCKWATCH_URL or GUARD_URL}) {json.dumps(dockwatch_config())}", flush=True)
    else:
        print("[dockwatch] off" + ("" if DOCKWATCH_ON else " (EBO_DOCKWATCH=0)"), flush=True)
    mqtt_host = os.environ.get("EBO_MQTT_HOST")
    if mqtt_host:
        try:
            from ebo_mqtt import EBOMqtt
            di = dict(DEVICE_INFO); di["ip_pi"] = lan_ip()
            EBOMqtt(bridge, do_action, do_move, di, mqtt_host,
                    int(os.environ.get("EBO_MQTT_PORT", "1883")),
                    os.environ.get("EBO_MQTT_USER") or None, os.environ.get("EBO_MQTT_PASS") or None)
            print(f"[supervisor] MQTT -> {mqtt_host}", flush=True)
        except Exception as e:
            print("[supervisor] MQTT error:", e, flush=True)
    uvicorn.run(app, host=BIND_HOST, port=PORT, log_level="warning")
