# LAN-only EBO registration responder

## Purpose and boundary

`tools/ebo_local_responder.py` supplies the narrow part of the Kalay control plane that the
tested EBO SE expects while its internet access is blocked. It binds UDP/10240 and UDP/3478 on
the bridge host, accepts only configured source addresses, validates the owned robot's
20-character UID, and emits registration responses built from the current request.

It does **not** implement AV, RDT, talkback, motor commands, cloud APIs, DNS, HTTPS, NTP, or a
general TUTK server. The normal `ebo_bridge` process continues to create the authenticated LAN
AV/RDT session. No DNS or Enabot health-name responder is required for the proven green-LED
path, so this service intentionally provides none.

## Protocol scope and the 125-second fix

The owned captures show two small exchanges:

| Destination | Accepted request | Generated response |
|---|---:|---:|
| UDP/10240 | 54-byte master lookup | 126-byte local endpoint list |
| UDP/3478 | 24-byte endpoint probe | 40-byte endpoint response |
| UDP/3478 | 128-byte device login | 44-byte login response |
| UDP/3478 | 48-byte registration/renewal | **56-byte registration ACK** |
| UDP/3478 | 4-byte `cc aa ab ab` keepalive | no response, matching capture |

The failed experiment used a captured 78-byte datagram for the 48-byte renewal. After decoding,
its header declared only 40 payload bytes (56 total), while 22 unrelated trailing bytes and stale
identity/endpoint material remained. The robot accepted enough of this malformed replay to turn
the LED green, but restarted registration roughly every 125 seconds. Each restart displaced the
bridge's Kalay session (`-20015`) and coincided with the robot turning left.

The service fixes that mechanism by constructing the same 56-byte response shape seen from the
real master: current configured UID, current robot endpoint, and the final four-byte nonce from
the current renewal. It never emits a 78-byte packet. UDP/10240 advertises the local responder's
address for both registration endpoints, avoiding continued dependence on remote relay IPs.
Malformed, unknown, wrong-UID, and wrong-source packets receive no response.

The packet fix is covered offline by a redacted 125-second trace fixture. Only a live coexistence
test can prove the robot firmware keeps the bridge session stable, so the service must not be
left unattended until the validation below passes.

## Configuration

Install the repository at `/opt/ebo-se-lan-bridge`, create an unprivileged `ebo-bridge` service
account, and keep the real configuration outside Git:

```bash
sudo install -d -o root -g root -m 0755 /opt/ebo-se-lan-bridge
sudo cp -a . /opt/ebo-se-lan-bridge/
sudo useradd --system --home-dir /nonexistent --shell /usr/sbin/nologin ebo-bridge 2>/dev/null || true
sudo install -o root -g ebo-bridge -m 0640 \
  tools/ebo-local-responder.env.example /etc/ebo-local-responder.env
sudoedit /etc/ebo-local-responder.env
```

Required values:

- `EBO_RESPONDER_BIND_IP`: bridge host's LAN address (`192.168.10.135` in this deployment).
- `EBO_RESPONDER_ROBOT_IP`: EBO address (`192.168.10.149`).
- `EBO_RESPONDER_ADVERTISE_IP`: normally the same LAN address as `BIND_IP`.
- `EBO_RESPONDER_UID`: exact 20-character UID of the owned robot; treat it as private.
- `EBO_RESPONDER_ALLOWED_SOURCES`: comma-separated robot and MikroTik LAN addresses. Hairpin
  masquerade makes redirected packets appear to come from the router; direct traffic advertised
  by the service comes from the robot.

Validate without opening sockets:

```bash
sudo -u ebo-bridge sh -c 'set -a; . /etc/ebo-local-responder.env; set +a; \
  /usr/bin/python3 /opt/ebo-se-lan-bridge/tools/ebo_local_responder.py --check-config'
```

## systemd installation

```bash
sudo install -o root -g root -m 0644 deploy/ebo-local-responder.service \
  /etc/systemd/system/ebo-local-responder.service
sudo systemctl daemon-reload
sudo systemctl enable --now ebo-local-responder.service
sudo systemctl status ebo-local-responder.service
sudo ss -lunp | grep -E ':(10240|3478)[[:space:]]'
```

The unit has no dependency on the bridge service. Start the responder first, then apply the
EBO-only router redirects, and restart neither service during the ten-minute observation window.

## MikroTik redirects

Juraj applies router changes separately. Do not paste these without first checking for existing
rules with the same comments. Substitute addresses if the deployment differs:

```routeros
/ip firewall nat add chain=dstnat src-address=192.168.10.149 protocol=udp dst-port=10240 action=dst-nat to-addresses=192.168.10.135 to-ports=10240 comment="EBO-local-TUTK-master"
/ip firewall nat add chain=srcnat src-address=192.168.10.149 dst-address=192.168.10.135 protocol=udp dst-port=10240 action=masquerade comment="EBO-local-TUTK-hairpin"
/ip firewall nat add chain=dstnat src-address=192.168.10.149 protocol=udp dst-port=3478 action=dst-nat to-addresses=192.168.10.135 to-ports=3478 comment="EBO-local-TUTK-3478"
/ip firewall nat add chain=srcnat src-address=192.168.10.149 dst-address=192.168.10.135 protocol=udp dst-port=3478 action=masquerade comment="EBO-local-TUTK-3478-hairpin"
```

Keep any WAN-block policy separate. Do not redirect all clients, DNS, TCP/443, or arbitrary UDP.
After changing NAT, inspect and remove only stale EBO UDP/10240 and UDP/3478 connection-tracking
entries if necessary; never clear connection tracking globally.

## Verification and acceptance gate

Before leaving this enabled:

1. Confirm `journalctl -fu ebo-local-responder` shows 54 -> 126, 24 -> 40,
   128 -> 44, and 48 -> 56 replies. Any 78-byte response is a release blocker.
2. Keep the ordinary bridge actively streaming/control-connected for more than ten minutes.
3. Confirm no repeating `session lost -20015` near 125-second intervals.
4. Confirm the robot does not turn or otherwise move without a requested motor command.
5. Confirm the status LED reaches and remains green.

If any session drop or unexplained movement occurs, disable the router rules immediately and
stop the responder. Offline tests prove packet construction, not firmware behaviour.

## Disable and remove

Stop the host service first:

```bash
sudo systemctl disable --now ebo-local-responder.service
```

Then disable the four MikroTik rules by exact comment:

```routeros
/ip firewall nat disable [find where comment="EBO-local-TUTK-master"]
/ip firewall nat disable [find where comment="EBO-local-TUTK-hairpin"]
/ip firewall nat disable [find where comment="EBO-local-TUTK-3478"]
/ip firewall nat disable [find where comment="EBO-local-TUTK-3478-hairpin"]
```

Remove only the EBO's affected UDP connection-tracking entries after inspection. The robot may
need one Wi-Fi reconnect or power cycle to discard old registration state. An orange status LED
is safer than retaining the responder if bridge stability or movement validation fails.
