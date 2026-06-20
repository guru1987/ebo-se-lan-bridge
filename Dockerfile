# EBO-SE bridge — runs on the Raspberry Pi (aarch64). The host kernel must allow 32-bit
# ARM execution (default on Raspberry Pi OS / HA OS); the bridge is a 32-bit Android
# binary run through the bundled bionic linker. Only the H.265 passthrough is done here,
# so a generic ffmpeg (no HW codecs) is enough.
ARG BUILD_FROM=python:3.11-slim-bookworm
FROM ${BUILD_FROM}

RUN apt-get update && apt-get install -y --no-install-recommends \
        ffmpeg jq curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir fastapi "uvicorn[standard]" paho-mqtt

WORKDIR /opt/ebo

# mediamtx (RTSP/WebRTC/HLS server)
ARG MEDIAMTX_VERSION=1.19.0
RUN set -e; \
    case "$(dpkg --print-architecture)" in \
      arm64) MT=arm64 ;; armhf) MT=armv7 ;; amd64) MT=amd64 ;; *) MT=arm64 ;; \
    esac; \
    curl -fsSL "https://github.com/bluenviron/mediamtx/releases/download/v${MEDIAMTX_VERSION}/mediamtx_v${MEDIAMTX_VERSION}_linux_${MT}.tar.gz" \
      | tar xz mediamtx

# our application code
COPY app/ /opt/ebo/
# vendor files (NOT in git): the 4 TUTK .so, the Android bionic runtime, and the stream-start blob.
# See docs/SETUP.md for how to populate ./vendor before building.
COPY vendor/lib/        /opt/ebo/lib/
COPY vendor/bionic/     /opt/ebo/bionic/
COPY vendor/ioctl9930.bin /opt/ebo/ioctl9930.bin

RUN chmod +x /opt/ebo/ebo_bridge /opt/ebo/bionic/linker /opt/ebo/run.sh /opt/ebo/mediamtx

ENV EBO_DIR=/opt/ebo EBO_PORT=8000
# 8000 web panel/API · 8554 RTSP · 8889 WebRTC · 8888 HLS
EXPOSE 8000 8554 8889 8888
CMD ["/opt/ebo/run.sh"]
