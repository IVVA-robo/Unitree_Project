# Offline LAN connection: Pico 4 Ultra, laptop, and Unitree R1

The project now has one local network path for the three devices. Internet,
cloud signaling, STUN, and TURN are not required.

```text
Pico 4 Ultra ── Wi‑Fi SSID Alaska ── laptop wlp4s0 192.168.8.9
                                      │
                                      └─ UDP discovery 9091 → commands 9090

laptop enxb4b024be59fe 192.168.123.162/24 ── Ethernet ── R1 PC2 192.168.123.164
                                      │
                                      └─ video-only Unitree videohub → :8080
```

The R1 native DDS/control endpoint documented by the hardware checks is
`192.168.123.161`; the PC2/SSH/voice computer is `192.168.123.164`. Both are
reachable through the same robot-facing link. The video source selects the
Ethernet interface for Unitree DDS, while the LAN preflight uses `.164` to
confirm that the direct PC2 link is present.

The NetworkManager profile `Unitree_R1` is static and autoconnects when the
Ethernet link appears. It is isolated from the Wi‑Fi default route. The R1 can
remain off while the laptop starts the local endpoints; the Unitree camera
source retries until the robot and its videohub are available.

Run the read-only audit at any time:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
./scripts/r1-lan-preflight
```

Start the automatic local session:

```bash
./scripts/r1-offline-session r1
```

This starts only the Pico UDP bridge and the video-only Robot POV server. It
does not start `r1_hardware_adapter`, `r1_live_writer`, `Sport`, `ArmSdk`,
locomotion, or head commands. For a laptop-only check use:

```bash
./scripts/r1-offline-session mock
```

The optional user service starts the same safe `r1` session after login and
restarts it if a local process exits:

```bash
make r1-offline-service-install
systemctl --user status r1-offline-session.service
systemctl --user stop r1-offline-session.service   # before a manual session
```

Avahi publishes the laptop hostname (currently `IONOS-ROBOTS.local`) on the
local LAN. The combined APK still
contains UDP bridge discovery, so a changed Wi‑Fi DHCP address does not require
rebuilding the control path. For a fully address-independent video endpoint,
build the combined APK against the local name:

```bash
UNITREE_LAN_DISCOVERY=mdns \
UNITREE_COMBINED_APK_PATH=/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller/Builds/CombinedDemo/UnitreeR1TelepresenceDemo_offline.apk \
./scripts/build-unity-telepresence build
```

If the Pico firmware cannot resolve mDNS, use the numeric URL printed by the
Robot POV server, or omit `UNITREE_LAN_DISCOVERY=mdns` and build with the
current Wi‑Fi address. Both transports
remain LAN-only.

The simulation remains a separate explicit mode:

```bash
systemctl --user stop r1-offline-session.service
./scripts/r1-offline-session sim
```

It keeps `hardware_enabled=false` and `ROS_LOCALHOST_ONLY=1`.
