# vendor/bionic/

The Android **bionic** runtime used to run the 32-bit ARM/Android TUTK libraries on the (glibc)
Raspberry Pi, via the Android dynamic linker. Pull these from a 32-bit Android system you own
(`adb pull /system/bin/linker`, `adb exec-out cat /system/lib/<name>`) — see
[../../docs/SETUP.md](../../docs/SETUP.md).

```
linker            (from /system/bin/linker, 32-bit)
libc.so
libm.so
libdl.so
libc++.so
liblog.so
libstdc++.so
libnetd_client.so
```
