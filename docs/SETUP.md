# Setup — populating `vendor/` and getting credentials

This integration needs the TUTK libraries, matching Android bionic runtime, and your robot
credentials. These are **device/account specific** and therefore not shipped in the repo.

## 1. TUTK libraries → `vendor/lib/`

The four shared libraries the bridge loads. Extract one matching ABI set from the ROLA APK
(`lib/armeabi-v7a/` for ARM32 or `lib/arm64-v8a/` for ARM64), for example with `unzip`:

```
vendor/lib/libTUTKGlobalAPIs.so
vendor/lib/libIOTCAPIs.so
vendor/lib/libRDTAPIs.so
vendor/lib/libAVAPIs.so
```

These must be SDK version **4.3.6.x** (Kalay 2.0, DTLS-PSK). Other devices/apps using the
same TUTK version also work.

## 2. Android bionic runtime → `vendor/bionic/`

The libraries are Android (bionic) binaries; the Linux host is glibc, so we run them with the
matching Android dynamic linker. For ARM32 pull `/system/bin/linker` and `/system/lib/*`.
For ARM64 pull `/system/bin/linker64` and `/system/lib64/*`, but store the selected linker as
`vendor/bionic/linker` so the container entrypoint remains ABI-neutral:

```
vendor/bionic/linker            (from /system/bin/linker or linker64)
vendor/bionic/libc.so
vendor/bionic/libm.so
vendor/bionic/libdl.so
vendor/bionic/libc++.so
vendor/bionic/liblog.so
vendor/bionic/libstdc++.so
vendor/bionic/libnetd_client.so
```

## 3. Optional stream-start blob → `vendor/ioctl9930.bin`

Some firmware needs the EBO-specific `avSendIOCtrl` `0x9930` payload captured from the app.
Save it as `vendor/ioctl9930.bin` when required. The tested SE1 firmware streams HEVC video
and G.711 audio without this blob; the bridge already treats a missing file as optional.

## 4. Credentials (into `.env`)

Extracted once from your installed ROLA app. Frida is not required: the device row is in
ROLA's private `history.db`, while the license and auth-key literals can be recovered from
the installed APK. Frida hooks remain an optional validation method.

| `.env` var | Source |
|------------|--------|
| `EBO_LICENSE` | `TUTK_SDK_Set_License_Key(...)` argument |
| `EBO_UID` | the 20-char TUTK UID |
| `EBO_AUTHKEY` | `St_IOTCConnectInput.authKey` (8 chars) |
| `EBO_IDENTITY` | `avClientStartEx` InConfig account (the UUID) |
| `EBO_TOKEN` | `avClientStartEx` InConfig password/token |

> The DTLS-PSK is derived by the library as `identity = "AUTHTKN_"+EBO_IDENTITY`,
> `psk = SHA256(EBO_TOKEN)` — you only need the two raw values above.

## 5. The bridge binary

A pre-compiled `app/ebo_bridge` (armeabi-v7a) is included. To rebuild it you need the
Android NDK. The script defaults to ARM32 and compiles both native source files:

```bash
NDK=/path/to/android-ndk bash scripts/build_bridge.sh
```

For an ARM64 Pixel or ODROID-C2, build to a separate path so the tracked ARM32 artifact is
not overwritten accidentally:

```bash
NDK=/path/to/android-ndk EBO_ARCH=arm64 \
  EBO_OUTPUT=app/ebo_bridge-arm64 bash scripts/build_bridge.sh
```

The bridge binary, TUTK libraries, bionic linker and bionic libraries must all use the same
ABI. An ARM64 Linux host such as the ODROID-C2 can run the ARM64 Android binary through the
bundled ARM64 bionic linker without QEMU.

Set `EBO_BRIDGE_BIN=ebo_bridge-arm64` in `.env` for that build. The suffixed binary is ignored
by Git as a local build artifact; do not commit it.

### x86-64 without Docker

The ARM64 bridge can also run through QEMU user-mode on an always-on x86-64 Linux host. This
does not create a VM or reserve guest RAM: only the native bridge process is translated, while
Python, ffmpeg and MediaMTX remain native x86-64 processes. Set:

```bash
EBO_BRIDGE_BIN=ebo_bridge-arm64
EBO_QEMU=/absolute/path/to/qemu-aarch64
```

Use an ARM64 bridge, ARM64 bionic linker/runtime and ARM64 TUTK libraries together. The
supervisor prepends `EBO_QEMU` when launching the bionic linker. Do not register a global
binfmt handler or install QEMU system-wide unless you explicitly want that behavior.

## Notes

- Only one client may stream from the robot at a time: close the ROLA app on your phone
  while the bridge is connected.
- An ARM32 deployment requires a host kernel with 32-bit ARM execution enabled.
- Browser push-to-talk requires HTTPS (or browser-trusted `localhost`) for microphone access.
- Only one web-panel talkback owner is accepted at a time.
