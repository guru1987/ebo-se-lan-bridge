# vendor/

Device/account-specific binaries the bridge needs at runtime. They are **not** shipped in this
repo (proprietary and/or specific to your device) — you provide them yourself from hardware and
software you own. Full instructions: [../docs/SETUP.md](../docs/SETUP.md).

Expected layout once populated:

```
vendor/
├── lib/                 # TUTK/Kalay shared libraries (see lib/README.md)
├── bionic/              # Android bionic runtime + linker (see bionic/README.md)
└── ioctl9930.bin        # captured "start streaming" avSendIOCtrl 0x9930 payload
```

Everything here except these `README.md` files is gitignored.
