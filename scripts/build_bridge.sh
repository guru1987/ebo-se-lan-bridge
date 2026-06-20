#!/usr/bin/env bash
# Rebuild the native bridge (armeabi-v7a / Android) from source.
# Requires the Android NDK. Set NDK to its path, or have the clang wrapper on PATH.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
: "${NDK:?Set NDK=/path/to/android-ndk}"
CLANG="$NDK/toolchains/llvm/prebuilt/$(uname | tr '[:upper:]' '[:lower:]')-x86_64/bin/clang"
"$CLANG" --target=armv7a-linux-androideabi24 -O2 "$HERE/app/ebo_bridge.c" -o "$HERE/app/ebo_bridge" -ldl
echo "built $HERE/app/ebo_bridge"
file "$HERE/app/ebo_bridge"
