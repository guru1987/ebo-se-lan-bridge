#!/usr/bin/env bash
# Rebuild the native bridge for Android/bionic.
# Requires the Android NDK. Set NDK to its path, or have the clang wrapper on PATH.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
: "${NDK:?Set NDK=/path/to/android-ndk}"
CLANG="$NDK/toolchains/llvm/prebuilt/$(uname | tr '[:upper:]' '[:lower:]')-x86_64/bin/clang"
# Preserve the repository's existing ARM32 deployment default.  Use
# EBO_ARCH=arm64 and preferably EBO_OUTPUT=... for Pixel/C2 builds so the
# tracked ARM32 binary is not overwritten accidentally.
ARCH="${EBO_ARCH:-armv7}"
OUTPUT="${EBO_OUTPUT:-$HERE/app/ebo_bridge}"

case "$ARCH" in
  arm64|aarch64) TARGET=aarch64-linux-android24 ;;
  armv7|armhf) TARGET=armv7a-linux-androideabi24 ;;
  *) echo "EBO_ARCH must be arm64/aarch64 or armv7/armhf" >&2; exit 2 ;;
esac

"$CLANG" --target="$TARGET" -std=c11 -O2 -Wall -Wextra \
  "$HERE/app/ebo_bridge.c" "$HERE/app/ebo_audio.c" \
  -o "$OUTPUT" -pthread -ldl
echo "built $OUTPUT ($TARGET)"
file "$OUTPUT"
