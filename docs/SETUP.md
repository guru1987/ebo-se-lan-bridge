# Setup — populating `vendor/` and getting credentials

This integration needs three things that are **device/account specific** and therefore not
shipped in the repo: the TUTK libraries, the Android bionic runtime, and your robot
credentials. You extract them once from your own EBO and the ROLA app.

## 1. TUTK libraries → `vendor/lib/`

The four shared libraries the bridge loads. Extract them from the ROLA APK
(`lib/armeabi-v7a/`) — e.g. with `apktool d rola.apk` or `unzip rola.apk 'lib/armeabi-v7a/*'`:

```
vendor/lib/libTUTKGlobalAPIs.so
vendor/lib/libIOTCAPIs.so
vendor/lib/libRDTAPIs.so
vendor/lib/libAVAPIs.so
```

These must be SDK version **4.3.6.x** (Kalay 2.0, DTLS-PSK). Other devices/apps using the
same TUTK version also work.

## 2. Android bionic runtime → `vendor/bionic/`

The libraries are Android (bionic) binaries; the host (Pi) is glibc, so we run them with the
Android dynamic linker. Pull these from any Android device with a 32-bit system
(`adb pull /system/bin/linker` and `adb exec-out cat /system/lib/<name> > vendor/bionic/<name>`):

```
vendor/bionic/linker            (from /system/bin/linker, 32-bit)
vendor/bionic/libc.so
vendor/bionic/libm.so
vendor/bionic/libdl.so
vendor/bionic/libc++.so
vendor/bionic/liblog.so
vendor/bionic/libstdc++.so
vendor/bionic/libnetd_client.so
```

## 3. Stream-start blob → `vendor/ioctl9930.bin`

The EBO-specific "start streaming" command (`avSendIOCtrl` 0x9930), captured once from the
app. Save the raw payload bytes to `vendor/ioctl9930.bin`.

## 4. Credentials (into `.env`)

Extracted once from the ROLA app while it connects to the robot (e.g. via Frida hooks on
`TUTK_SDK_Set_License_Key`, `IOTC_Connect_ByUIDEx`'s `St_IOTCConnectInput`, and
`avClientStartEx`'s config struct):

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

A pre-compiled `app/ebo_bridge` (armeabi-v7a) is included. To rebuild it from
`app/ebo_bridge.c` you need the Android NDK:

```bash
clang --target=armv7a-linux-androideabi24 -O2 app/ebo_bridge.c -o app/ebo_bridge -ldl
```

(see `scripts/build_bridge.sh`).

## Notes

- Only one client may stream from the robot at a time: close the ROLA app on your phone
  while the bridge is connected.
- The host kernel must allow 32-bit ARM execution (default on Raspberry Pi OS / HA OS).
