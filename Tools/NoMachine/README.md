# NoMachine tools

Publication note (2026-09-30): private home/session trees, extracted runtimes
and newly inventoried proprietary installers are not included in this GitHub
export. The launchers below require a separately obtained, licensed runtime.
They are retained as configuration/history, not a ready-to-run public install.

Portable NoMachine clients used to access the Unitree R1 onboard computer.

- The system installation in `/usr/NX` remains unchanged.
- Portable clients are kept in versioned subdirectories.
- No robot motion or ROS control is started by these tools.

## NoMachine 9.7.3 portable

`9.7.3-portable/run-nxplayer-9.7.3.sh` starts an isolated NoMachine 9.7.3
client using Bubblewrap. It has its own home directory and maps the extracted
runtime to `/usr/NX` only inside the sandbox. The system installation is not
modified.

The source is the NoMachine 9.7.3 full Debian package retained by the signed
ITEAS Ubuntu repository:

- file: `nomachine_9.7.3_1_amd64.deb`
- package URL:
  `https://apt.iteas.at/iteas/pool/main/n/nomachine/nomachine_9.7.3_1_amd64.deb`
- signed repository metadata:
  `https://apt.iteas.at/iteas/dists/jammy/InRelease`
- size: `81483838` bytes
- SHA-256:
  `81e0f8b48c7a4d3c16dac0401b9d188353e4a237498284d267a05c5176fd95fc`
- SHA-512:
  `a3b03c016c0c45cf8d833f161fb53b157963258a5d8d9006dd3f0fb2c03ce9a5bd45600c0351c627e0edfe3ed6311aa535c169820aaba5b92f4d3772e46b7d8e`
- repository signing-key fingerprint:
  `BA66 2621 DA69 F38C 443F 147C 23CA E455 82EB 0928`

Only the bundled `nxrunner` and `nxplayer` payloads were extracted. No package
maintainer script, NX server installer, node, or service was executed.
An offline launcher smoke test returned `NXPLAYER - Version 9.7.3` on
2026-09-14; no robot connection was made.

Use `9.7.3-portable/connect-r1.sh` for the R1 A/B test. As with the 9.8.2
launcher, it validates the dedicated Ethernet interface, source address,
route, and NoMachine TCP port before starting the client. The copied profile
contains the username and accepted host certificate but no password.

## NoMachine 9.8.2 portable

`9.8.2-portable/run-nxplayer-9.8.2.sh` starts the client in a Bubblewrap
container and maps its runtime to `/usr/NX`. The client has a private home
directory, so it cannot overwrite the configuration of the system-wide
NoMachine 10 client.

The source package is the official NoMachine 9.8.2 Linux x86_64 TAR archive:

- file: `nomachine_9.8.2_1_x86_64.tar.gz`
- official SHA-512 from the AUR release history:
  `2fd993636543e143a94b6b0a67e47cd2ea195b5a9bd989bf64a978151630403911407b88afb4ebb452b8b35d4698429ff890413d9a49f1487cf28e22071f1eeb`
- local SHA-256:
  `6e95d8784ae3c78faca31b1223a10c3b495f15410901ce584f76487e44dd441a`

Only the `nxrunner` and `nxplayer` payloads were extracted. The bundled NX
server, node and service installers were not installed or executed.

The text installation templates `player.cfg` and `runner.cfg` intentionally
remain outside `runtime/NX/etc`: NoMachine installs those files under
`/etc/NX/server/localhost`, while `/usr/NX/etc/player.cfg` is interpreted as
an XML GUI settings file. The portable client's valid XML settings are kept
under `9.8.2-portable/home/.nx/config`. An offline GUI smoke test completed
successfully on 2026-09-14 with NoMachine 9.8.2; no robot connection was made.

Use `9.8.2-portable/connect-r1.sh` for the R1 connection. It validates that
`192.168.123.164` is reached directly through `enxb4b024be59fe` from
`192.168.123.162` and that TCP port 4000 is open before starting NoMachine.
This prevents an accidental attempt through Wi-Fi when the robot cable is
disconnected.

The portable profile and the already accepted public host certificate are
kept under `9.8.2-portable/home/.nx`. The profile stores the username but does
not store a password (`Remember password=false`, `EMPTY_PASSWORD`).
