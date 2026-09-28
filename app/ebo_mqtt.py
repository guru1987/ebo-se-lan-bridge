"""
Home Assistant MQTT Discovery for the EBO-SE bridge.

Publishes a single "EBO-SE" device with control buttons/switches, battery sensors
and diagnostic sensors. HA auto-creates all entities. Commands received over MQTT
are forwarded to the robot through the bridge (RDT / MAVLink).

The bridge holds the robot credentials; Home Assistant only ever talks to MQTT.
"""
import json, threading, time
import paho.mqtt.client as mqtt

DISCOVERY_PREFIX = "homeassistant"
BASE = "ebo"
AVAIL = f"{BASE}/availability"

DEVICE = {
    "identifiers": ["ebo_se_bridge"],
    "name": "EBO-SE",
    "model": "EBO SE",
    "manufacturer": "Enabot",
}

# Buttons: command topic .../cmd
BUTTONS = [("wake", "Wake", "mdi:white-balance-sunny"),
           ("sleep", "Sleep", "mdi:sleep"),
           ("dock", "Dock", "mdi:home-import-outline"),
           ("undock", "Stop docking", "mdi:hand-back-right")]
# Movement buttons (handy for automations)
MOVES = [("forward", "Forward", "mdi:arrow-up-bold"),
         ("backward", "Backward", "mdi:arrow-down-bold"),
         ("left", "Left", "mdi:arrow-left-bold"),
         ("right", "Right", "mdi:arrow-right-bold")]
# Switches (optimistic): command .../set, state .../state
SWITCHES = [("eyes", "Eye lights", "mdi:eye"),
            ("night", "Night vision", "mdi:weather-night"),
            ("avoid", "Collision avoidance", "mdi:shield-alert"),
            ("fall", "Fall protection", "mdi:stairs"),
            ("patrol", "Patrol", "mdi:robot")]

NUMBERS = {
    "speaker_volume": ("Speaker volume", "mdi:volume-high", "audio-volume", 0, 100, 1),
    "call_volume": ("Call volume", "mdi:account-voice", "audio-call_volume", 0, 100, 1),
    "drive_speed": ("Drive speed", "mdi:speedometer", "control-speed", 0, 100, 1),
    "security_sensitivity": ("Security sensitivity", "mdi:motion-sensor", "security_fixed-sensitivity", 0, 4, 2),
    "notification_interval": ("Notification interval", "mdi:timer-outline", "security_fixed-notify_interval", -1, 3600, 1),
}

SWITCH_PARAMETERS = {
    "eyes": "display-enable", "night": "video-night_vision",
    "avoid": "control-auto_avoidance", "fall": "control-fallarrest",
    "patrol": "security_patrol-enable",
}

TELEMETRY_SENSORS = {
    "battery/measured_voltage": ("Battery voltage", "V", "voltage"),
    "battery/current": ("Battery current", "A", "current"),
    "battery/estimated_voltage": ("Estimated battery voltage", "V", "voltage"),
    "battery/charge_voltage": ("Charge voltage", "V", "voltage"),
    "attitude/roll": ("Roll", "rad", None),
    "attitude/pitch": ("Pitch", "rad", None),
    "attitude/yaw": ("Yaw", "rad", None),
    "acceleration/x": ("Acceleration X", None, None),
    "acceleration/y": ("Acceleration Y", None, None),
    "acceleration/z": ("Acceleration Z", None, None),
    "gyroscope/x": ("Gyroscope X", None, None),
    "gyroscope/y": ("Gyroscope Y", None, None),
    "gyroscope/z": ("Gyroscope Z", None, None),
    "motor_speed/left": ("Left motor speed", None, None),
    "motor_speed/right": ("Right motor speed", None, None),
    "motor_temperature/left": ("Left motor temperature", "°C", "temperature"),
    "motor_temperature/right": ("Right motor temperature", "°C", "temperature"),
}


class EBOMqtt:
    def __init__(self, bridge, do_action, do_move, device_info, host, port=1883, user=None, pw=None):
        self.bridge = bridge
        self.do_action = do_action      # do_action(name)
        self.do_move = do_move          # do_move(ly, rx)
        self.diag = device_info         # static dict
        self.c = (mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
                  if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client())
        if user:
            self.c.username_pw_set(user, pw)
        self.c.will_set(AVAIL, "offline", retain=True)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.c.connect(host, port, 60)
        self.c.loop_start()
        bridge.on_status = self._on_status
        bridge.on_state = self._on_state
        threading.Thread(target=self._state_loop, daemon=True).start()

    def _d(self, comp, obj, cfg):
        cfg.setdefault("availability_topic", AVAIL)
        cfg.setdefault("device", DEVICE)
        cfg.setdefault("unique_id", f"ebo_{obj}")
        self.c.publish(f"{DISCOVERY_PREFIX}/{comp}/{BASE}/{obj}/config", json.dumps(cfg), retain=True)

    def _on_connect(self, *a):
        self.c.publish(AVAIL, "online", retain=True)
        # controls
        for k, n, ic in BUTTONS + MOVES:
            self._d("button", k, {"name": n, "icon": ic, "command_topic": f"{BASE}/{k}/cmd", "payload_press": "PRESS"})
        for k, n, ic in SWITCHES:
            self._d("switch", k, {"name": n, "icon": ic, "command_topic": f"{BASE}/{k}/set",
                                  "state_topic": f"{BASE}/{k}/state", "payload_on": "ON", "payload_off": "OFF"})
        for key, (name, icon, _, minimum, maximum, step) in NUMBERS.items():
            self._d("number", key, {"name": name, "icon": icon,
                    "command_topic": f"{BASE}/{key}/set", "state_topic": f"{BASE}/{key}/state",
                    "min": minimum, "max": maximum, "step": step, "mode": "slider"})
        # battery / charging / connectivity
        self._d("sensor", "battery", {"name": "Battery", "state_topic": f"{BASE}/battery",
                                      "unit_of_measurement": "%", "device_class": "battery"})
        self._d("binary_sensor", "charging", {"name": "Charging", "state_topic": f"{BASE}/charging",
                                              "payload_on": "ON", "payload_off": "OFF", "device_class": "battery_charging"})
        self._d("binary_sensor", "online", {"name": "Connected", "state_topic": f"{BASE}/online",
                                            "payload_on": "ON", "payload_off": "OFF", "device_class": "connectivity",
                                            "entity_category": "diagnostic"})
        # Awake = camera is streaming (robot not in sleep). Drives the HA fallback image.
        self._d("binary_sensor", "awake", {"name": "Awake", "state_topic": f"{BASE}/awake",
                                           "payload_on": "ON", "payload_off": "OFF", "device_class": "running"})
        for path, (name, unit, device_class) in TELEMETRY_SENSORS.items():
            key = "telemetry_" + path.replace("/", "_")
            cfg = {"name": name, "state_topic": f"{BASE}/telemetry/{path}",
                   "entity_category": "diagnostic", "state_class": "measurement"}
            if unit:
                cfg["unit_of_measurement"] = unit
            if device_class:
                cfg["device_class"] = device_class
            self._d("sensor", key, cfg)
        # static diagnostics
        for key, name in [("name", "Name"), ("model", "Model"), ("sn", "Serial"), ("fsn", "FSN"),
                          ("mcu_version", "MCU version"), ("camera_version", "Camera firmware"),
                          ("uid", "TUTK UID"), ("wifi_ssid", "Wi-Fi SSID"), ("ip", "IP"), ("mac", "MAC")]:
            val = self.diag.get(key)
            if not val:
                continue
            self._d("sensor", key, {"name": name, "state_topic": f"{BASE}/diag/{key}", "entity_category": "diagnostic"})
            self.c.publish(f"{BASE}/diag/{key}", str(val), retain=True)
        # informational: RTSP URL (HA RTSP cameras are added via the Generic Camera integration)
        self._d("sensor", "rtsp", {"name": "RTSP stream", "state_topic": f"{BASE}/diag/rtsp",
                                   "entity_category": "diagnostic", "icon": "mdi:cctv"})
        self.c.publish(f"{BASE}/diag/rtsp", f"rtsp://{self.diag.get('ip_pi', '<pi>')}:8554/ebo", retain=True)
        # subscribe
        self.c.subscribe(f"{BASE}/+/cmd")
        self.c.subscribe(f"{BASE}/+/set")
        threading.Thread(target=self.bridge.refresh_parameters, daemon=True).start()

    def _on_message(self, c, u, msg):
        parts = msg.topic.split("/")
        if len(parts) < 3:
            return
        key, kind = parts[1], parts[2]
        payload = msg.payload.decode(errors="replace")
        if kind == "cmd":
            if key in ("wake", "sleep", "dock", "undock"):
                self.do_action(key)
            elif key == "forward": self.do_move(0.7, 0)
            elif key == "backward": self.do_move(-0.7, 0)
            elif key == "left": self.do_move(0, -0.7)
            elif key == "right": self.do_move(0, 0.7)
        elif kind == "set":
            if key in NUMBERS:
                try:
                    parameter = NUMBERS[key][2]
                    group, param_key = parameter.split("-", 1)
                    self.bridge.set_parameter(group, param_key, float(payload))
                except (ValueError, RuntimeError):
                    pass
                return
            on = (payload.upper() == "ON")
            self.do_action(f"{key}_{'on' if on else 'off'}")

    def _on_state(self, kind, name, value):
        if kind == "telemetry" and isinstance(value, dict):
            for field, field_value in value.items():
                path = f"{name}/{field}"
                if path in TELEMETRY_SENSORS:
                    self.c.publish(f"{BASE}/telemetry/{path}", str(field_value), retain=True)
            return
        if kind != "parameter":
            return
        for key, parameter in SWITCH_PARAMETERS.items():
            if parameter == name:
                self.c.publish(f"{BASE}/{key}/state", "ON" if float(value) else "OFF", retain=True)
        for key, (_, _, parameter, _, _, _) in NUMBERS.items():
            if parameter == name:
                self.c.publish(f"{BASE}/{key}/state", str(round(float(value), 3)), retain=True)

    def _on_status(self, battery, charge):
        self.c.publish(f"{BASE}/battery", str(battery), retain=True)
        self.c.publish(f"{BASE}/charging", "ON" if charge == 1 else "OFF", retain=True)

    def _state_loop(self):
        last = None; last_awake = None
        while True:
            st = "ON" if self.bridge.connected else "OFF"
            if st != last:
                self.c.publish(f"{BASE}/online", st, retain=True); last = st
            awake = "ON" if self.bridge.is_awake() else "OFF"
            if awake != last_awake:
                self.c.publish(f"{BASE}/awake", awake, retain=True); last_awake = awake
            if self.bridge.battery >= 0:
                self.c.publish(f"{BASE}/battery", str(self.bridge.battery), retain=True)
                self.c.publish(f"{BASE}/charging", "ON" if self.bridge.charge == 1 else "OFF", retain=True)
            time.sleep(5)
