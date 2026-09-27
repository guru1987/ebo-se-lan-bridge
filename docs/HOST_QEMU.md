# Native x86-64 host with QEMU user-mode

This layout runs Python, ffmpeg and MediaMTX natively on x86-64. Only the small ARM64 Android
bridge process is translated by `qemu-aarch64`. It does not create a virtual machine, reserve
guest RAM, register binfmt handlers, require root, or use Docker.

Recommended workspace:

```text
~/ebo_bridge/
├── repo/                 # this Git repository
├── runtime/              # ignored/private runtime and credentials
│   ├── .env              # mode 0600
│   ├── ebo_bridge-arm64
│   ├── mediamtx
│   ├── bionic/           # ARM64 Android linker and runtime
│   └── lib/              # owned ARM64 TUTK libraries
├── tools/qemu/           # privately unpacked qemu-user package
└── venv/                 # FastAPI/uvicorn/paho-mqtt
```

Start it with:

```bash
cd ~/ebo_bridge/repo
bash scripts/run_host_qemu.sh
```

The default web bind is loopback. For a headless host that should serve the local LAN and
WireGuard network, set `EBO_BIND=0.0.0.0` in the private runtime `.env`; then access it at
`http://<host-lan-address>:8000/` or `http://<host-wireguard-address>:8000/`. Restrict port
8000 with the host firewall to the intended subnets (`192.168.10.0/24` and `10.22.33.0/24`)
when other host interfaces are not trusted.

If the panel should remain private, use an SSH tunnel instead:

```bash
ssh -L 8000:127.0.0.1:8000 user@bridge-host
```

Then open `http://localhost:8000/`. Browsers treat `localhost` as a secure context for
microphone capture. For permanent remote access, use an authenticated HTTPS reverse proxy.

The ARM64 bridge, bionic runtime and TUTK libraries must come from the same owned ROLA/Android
ABI set. None belongs in Git. `ioctl9930.bin` is also private and is needed only by firmware
that does not start video with the bridge's standard IOCTRL sequence; the tested SE1 streams
without it.
