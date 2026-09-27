#!/usr/bin/env bash
# Run the EBO supervisor natively on x86-64 while translating only the ARM64
# Android bridge with QEMU user-mode. No Docker, VM, or binfmt registration.
set -euo pipefail
umask 077

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORKSPACE="${EBO_WORKSPACE:-$(cd "$REPO/.." && pwd)}"
RUNTIME="${EBO_RUNTIME_DIR:-$WORKSPACE/runtime}"
VENV="${EBO_VENV_DIR:-$WORKSPACE/venv}"
ENV_FILE="${EBO_ENV_FILE:-$RUNTIME/.env}"

for path in \
  "$ENV_FILE" \
  "$RUNTIME/ebo_bridge-arm64" \
  "$RUNTIME/bionic/linker" \
  "$RUNTIME/mediamtx" \
  "$VENV/bin/python"; do
  if [[ ! -e "$path" ]]; then
    echo "missing required host-runtime file: $path" >&2
    exit 2
  fi
done

set -a
# The file is local, mode 0600, and contains shell-style KEY=VALUE entries.
source "$ENV_FILE"
set +a

export EBO_DIR="$RUNTIME"
export EBO_BRIDGE_BIN="${EBO_BRIDGE_BIN:-ebo_bridge-arm64}"
export EBO_QEMU="${EBO_QEMU:-$WORKSPACE/tools/qemu/usr/bin/qemu-aarch64}"
export EBO_LIB_DIR="$RUNTIME/lib"
export EBO_IOCTL9930="$RUNTIME/ioctl9930.bin"

: "${EBO_STREAM_USER:=ebo}"
: "${EBO_STREAM_PASS:?missing EBO_STREAM_PASS in $ENV_FILE}"

sed -e "s/__STREAM_USER__/${EBO_STREAM_USER}/g" \
    -e "s/__STREAM_PASS__/${EBO_STREAM_PASS}/g" \
    "$REPO/app/mediamtx.template.yml" > "$RUNTIME/mediamtx.yml"
chmod 600 "$RUNTIME/mediamtx.yml"

exec "$VENV/bin/python" "$REPO/app/ebo_server.py"
