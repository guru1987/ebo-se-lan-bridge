#!/usr/bin/env bash
# Entrypoint for the EBO-SE bridge container.
# Reads Home Assistant add-on options (/data/options.json) if present, otherwise uses env vars.
# Credentials are NEVER baked into the image: they come from the add-on config / env.
set -e
APP=/opt/ebo
OPT=/data/options.json

# When run as a Home Assistant add-on, map options.json -> env
if [ -f "$OPT" ] && command -v jq >/dev/null 2>&1; then
  export EBO_LICENSE=$(jq -r '.license_key // empty' "$OPT")
  export EBO_UID=$(jq -r '.uid // empty' "$OPT")
  export EBO_AUTHKEY=$(jq -r '.authkey // empty' "$OPT")
  export EBO_IDENTITY=$(jq -r '.av_identity // empty' "$OPT")
  export EBO_TOKEN=$(jq -r '.av_token // empty' "$OPT")
  export EBO_STREAM_USER=$(jq -r '.stream_user // "ebo"' "$OPT")
  export EBO_STREAM_PASS=$(jq -r '.stream_pass // empty' "$OPT")
  export EBO_WEB_USER=$(jq -r '.web_user // empty' "$OPT")
  export EBO_WEB_PASS=$(jq -r '.web_pass // empty' "$OPT")
  export EBO_MQTT_HOST=$(jq -r '.mqtt_host // empty' "$OPT")
  export EBO_MQTT_PORT=$(jq -r '.mqtt_port // 1883' "$OPT")
  export EBO_MQTT_USER=$(jq -r '.mqtt_user // empty' "$OPT")
  export EBO_MQTT_PASS=$(jq -r '.mqtt_pass // empty' "$OPT")
fi

: "${EBO_LICENSE:?missing license_key}"; : "${EBO_UID:?missing uid}"; : "${EBO_AUTHKEY:?missing authkey}"
: "${EBO_IDENTITY:?missing av_identity}"; : "${EBO_TOKEN:?missing av_token}"
: "${EBO_STREAM_USER:=ebo}"; : "${EBO_STREAM_PASS:?missing stream_pass (RTSP password)}"

# Render the mediamtx config with the stream credentials
sed -e "s/__STREAM_USER__/${EBO_STREAM_USER}/g" -e "s/__STREAM_PASS__/${EBO_STREAM_PASS}/g" \
    "$APP/mediamtx.template.yml" > "$APP/mediamtx.yml"

cd "$APP"
# A Docker container already provides an isolated PID namespace (low PIDs), which the
# 32-bit bionic libc requires (pid <= 65535). No 'unshare' needed inside a container.
exec python3 "$APP/ebo_server.py"
