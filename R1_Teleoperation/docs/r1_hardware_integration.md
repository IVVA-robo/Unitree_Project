# R1 hardware integration readiness

This note records the read-only checks, verified Unitree SDK contract, and the
source-backed `r1_live_writer` safety boundary. The supported operational mode
is still mock/read-only/dry-run. A staged physical `StandUp` has now confirmed
stable FSM `4`; locomotion was not repeated after the first post-prepare
`SetVelocity` returned error `127`. The R1 example requires `Start()` and
stable FSM `811` before velocity calls, and the writer now enforces that
sequence. Head output did not start after LowState reported
`pitch=0.0073 rad`, `yaw=1.4299 rad`. The physical pose/direction and its
joint-zero correlation were not visually confirmed.
`r1_live_writer` is implemented, but
its physical-session wrappers are fail-closed: without the full commissioning
acknowledgements they refuse to start an SDK/DDS writer.

## Startup head correlation (2026-09-17)

After the carabiners were removed from the head workspace, the operator
confirmed that the R1 itself turns its head sideways on every boot. During that
visible boot pose no local command writer was running. A 501-sample read-only
LowState observation was nearly stationary and reported:

```text
head_yaw mean=2.00629520 rad, last=2.00629950 rad, span=0.00003314 rad
head_yaw dq max=0.02290078 rad/s
head_pitch mean=-0.00372654 rad
```

The yaw coordinate is very close to the documented positive joint limit
`+2.0071 rad`. Unitree's official
[R1 Startup Video on Bracket](https://support.unitree.com/home/en/R1_developer/bracket_start)
explicitly says, “The head rotates to the left into calibration position,” and
shows it remaining turned at the end. This correlates the encoder value with
the documented startup/firmware calibration pose rather than a VR or
local-writer command. It does **not** establish that numerical
`q=0` is a safe physical centre. The earlier `1.43..1.46 rad` samples were
taken while the head workspace may have been obstructed by the suspension
carabiners and must not be used as a centre reference. The head workspace must
be clear before every power-on; never let a cable, carabiner, support, person,
or tool oppose the automatic boot motion.

## Read-only result (2026-09-16)

- Robot-facing interface: `enxb4b024be59fe`, `192.168.123.162/24`.
- Native DDS endpoint: `192.168.123.161`; PC2: `192.168.123.164`.
- ICMP and TCP/22 + TCP/4000 probes succeeded.
- DDS discovery found `rt/lf/lowstate`, `rt/frontvideostream`, the videohub
  request/response pair, and the R1 arm/locomotion services.
- LowState monitor received fresh samples (about 1 ms age): 35 motor slots,
  26 configured R1 joints, `motorstate=none`, `max_abs_dq=0.016`,
  `max_abs_tau_est=0.070`, temperatures 24..39, and `mode_machine=1`.
- Video-only `VideoClient.GetImageSample()` returned 1280x720 JPEGs; the
  viewer produced a live 640x360 stream at roughly 12 FPS on a temporary port.

No motor command channel or locomotion RPC was called during these checks.

## Dry-run arm handoff result (2026-09-16)

With the R1 connected, `r1_hardware_dry_run.launch.py` was started in an
isolated ROS domain (`ROS_DOMAIN_ID=89`). A synthetic, symmetric headset pose
was streamed for four seconds and then Deadman was released. The chain

```text
/r1/hardware/joint_states -> headset-relative IK -> safety gate -> debug trajectory
```

produced mirrored targets in `torso_link`:

```text
left  (x=0.128371, y=+0.138605, z=-0.018039)
right (x=0.128371, y=-0.138605, z=-0.018039)
```

The dry-run adapter reported `hardware_enabled=false`; no `rt/arm_sdk`,
`rt/lowcmd`, `sport`, or controller command topic was opened. This confirms
the transform layer is symmetric before physical commissioning review.

## Verified SDK contract

The SDK source is installed at
`/home/unitree/Unitree_Project/ROS2_WS/unitree_sdk2`.

### Arms and head

The official example
`example/r1/high_level/r1_arm_sdk_dds_example.cpp` uses
`unitree::robot::r1::publisher::ArmSdk` on `rt/arm_sdk`.  Its ordered joints
and HG IDL slots are:

| command order | joint | IDL slot |
|---:|---|---:|
| 0..4 | left shoulder pitch/roll/yaw, elbow, wrist roll | 15..19 |
| 5..9 | right shoulder pitch/roll/yaw, elbow, wrist roll | 22..26 |
| 10 | waist yaw | 13 |
| 11 | head pitch | 29 |
| 12 | head yaw | 30 |

The example ramps `mode_pr` (SDK weight) to 1.0, seeds every target from the
measured LowState position, sets bounded `kp/kd`, and releases the weight back
to zero. `r1_live_writer` preserves this hand-over sequence in its SDK
implementation and must never take control merely because a ROS node started.
Its physical behavior has not yet been validated on the R1.

The built-in R1 Arm Action service also owns a DDS writer endpoint on
`rt/arm_sdk`. Unitree explicitly forbids running an Arm Action and a custom
`ArmSdk` command stream at the same time; an occupied channel is reported as
error `7400`. The same documentation says that the service itself does not
need to be stopped when no Arm Action API is being called. Therefore discovery
of one persistent publication is not, by itself, evidence of competing
commands and must not be confused with the robot-side subscription that
consumes `rt/arm_sdk`.

The commissioning gate now uses typed read-only evidence instead: it watches
for actual `LowCmd` samples on `rt/arm_sdk` and checks the built-in action-state
topic `rt/arm/action/state`. During the 2026-09-17 read-only investigation the
command observer received zero `LowCmd` samples in six seconds, while the
typed action state was idle:

```json
{"holding": false, "id": 0, "name": ""}
```

This is evidence only for that bounded observation window, not permanent
permission to start a writer. Any received command sample, `holding=true`, a
non-zero/non-empty action, missing or malformed action state, observer/ABI
failure, or ambiguous discovery result blocks commissioning. The check must be
repeated immediately before a physical session and again at the final
one-shot probe boundary. Official reference:
<https://support.unitree.com/home/en/R1_developer/arm_control_routine>.

The observer now retains the public typed `CreateRecvChannel<LowCmd_>` result
and obtains the native reader from it. Because this vendor CycloneDDS `0.10.2`
build reports the ISO C++ `matched_publications()` operation as unsupported,
the observer uses the corresponding read-only C API on that same typed reader.
Exit `0` requires the explicit phase-specific set: one built-in publication
before local transport initialization, or two publications after our known
local ArmSdk writer has been constructed for the ownership probe. Matching
topic/type metadata, the same complete handle set, cumulative match count equal
to the expected count, and at least a final two-second churn-free interval are
required. Missing, extra, replaced or churning writers return exit `15`. This
closes the unmatched-reader ambiguity, but exit `0` remains only one required
interlock and is never standalone authorization for a physical probe.

The local simulation URDF contains the 24 movable body/arm joints but no head
links.  The physical LowState has two additional head joints.  A physical
deployment therefore needs a reviewed URDF/controller map for those joints;
the simulation URDF must not be treated as the complete hardware model.

### Locomotion

The official
`example/r1/high_level/r1_loco_client_example.cpp` uses `r1::LocoClient`:

- service: `sport`;
- API version: `1.0.0.0`;
- velocity API: `SetVelocity(vx, vy, omega, duration)` (API 7105);
- safety stop: `StopMove()` sends a zero-velocity command;
- mode changes include `Start`, `Damp`, `StandUp`, and `ZeroTorque`.

The writer owns an independent watchdog and calls `StopMove()` on deadman
release, stale packet, invalid command, DDS disconnect, kill, or shutdown.
No raw leg-joint command is used: `rt/lowcmd` is forbidden, and no
mode-changing call belongs in the upstream teleoperation node.

## DDS ABI incident, containment, and first staged result (2026-09-17)

An earlier acknowledged head-only prepare attempt did not move the robot.
`r1_live_writer_node` terminated synchronously during Unitree SDK transport
initialization, before the asynchronous worker and before `StandUp()`. The
independent supervisor remained fail-closed and asserted its kill latch.

The loader had combined ROS Humble CycloneDDS `libddsc.so.0` (0.10.5) with
Unitree's `libddscxx.so.0` (0.10.2). An isolated reproduction produced
`free(): invalid pointer`; the matched vendor pair exited normally. The
physical runtime now uses two deliberately separate middleware families:

- ROS graph: `rmw_fastrtps_cpp`;
- Unitree SDK2: its matched vendor `libddsc.so.0` and `libddscxx.so.0`.

`r1-sdk-preflight` runs `r1-sdk-abi-probe` before starting a robot-facing
reader. The probe enters a private user/network namespace, verifies that only
`lo` exists, checks both resolved library paths, constructs the ROS and SDK
objects, and exercises teardown without calling any command method. The writer
also validates the active RMW and both loaded library paths before
`ChannelFactory::Init`. A mismatch fails closed. This containment passed the
isolated positive and deliberately mixed-library negative probes.

After containment, the official `StandUp` completed and `GetFsmId()` was
stably `4`. The required Deadman release/re-press then reached the head seed
gate. Fresh LowState was `head_pitch=0.0073 rad`, `head_yaw=1.4299 rad`, and
the encoder yaw value is numerically about `81.9°` in joint coordinates. This
is not an independent visual measurement: the physical pose, direction, and
joint-zero correlation remain unconfirmed. In that first normal-tracking
attempt the writer latched kill before `seed_head`; no ArmSdk head frame was
published. In a later separately authorized recenter-only attempt, however,
ArmSdk was seeded from fresh feedback near `yaw=1.4606`, `pitch=0.0084` and
full-weight frames were queued. Feedback did not follow the small target, so
`head_recenter_following_timeout` stopped the trajectory, held the fresh
feedback pose, released ownership to `weight=0`, and latched kill. The user
then observed a red chest indicator. This distinction matters: the later log
proves that the local ArmSdk publisher boundary was crossed, but it does not
prove that robot firmware accepted or acted on those frames. The old code
incorrectly reused the small headset-relative
limit as an absolute joint limit. Those concepts are now separated and covered
offline, while normal tracking deliberately retains a centred seed window.
The old full-zero algorithm remains available only for offline unit comparison;
all physical recenter entry points now fail closed. A replacement ownership
micro-probe passes loopback-only MockTransport coverage for passive weight-zero
discovery, a bounded weight ramp, a yaw excursion of at most `0.005 rad`,
same-direction feedback, return to the exact seed, weight-zero release,
an independent `0.04 rad/s` measured-speed ceiling, disarm, and kill. This does
not authorize physical use. The
next on-robot step is blocked until the red indicator is understood. Once the
robot is healthy, it must begin with a fresh read-only visual correlation and a
separately approved ownership micro-probe, not the full recenter trajectory. If
the head appears straight while feedback remains near `1.4299`, do not command
zero and investigate joint indexing/zero semantics first.

## Implemented code and remaining commissioning tasks

### Read-only transport

`ros2_ws/src/r1_sdk_transport` is a separate vendor-SDK C++ LowState reader.
With `sdk_enabled=true` it creates exactly one reader for `rt/lf/lowstate` on
the configured Ethernet interface and republishes the 26 physical joints under
`/r1/sdk/joint_states`. It verifies fresh finite `q`/`dq` values before
publishing. It also continuously publishes
`/r1/sdk_transport/motors_healthy`: true requires a fresh valid sample and
`motorstate==0` in every configured physical slot; diagnostics retain every
nonzero `slot:code`. It has no DDS writer or locomotion client; it is still the only
permitted robot-facing process during read-only review.

The simulation URDF's 24 joints are not silently used as the physical map: the
reader configuration retains all 26 physical names, including the two head
joints. A first accepted debug arm target is seeded from feedback and shaped by
position, slew, rad/s, Deadman, and command-watchdog limits before it is copied
to `/r1/sdk_transport/debug/arm_trajectory`.

### Isolated writer architecture

`ros2_ws/src/r1_live_writer` consumes only already-shaped diagnostic ROS input:

```text
/r1_hardware_adapter/debug/head/joint_trajectory
/r1/locomotion_dry_run/debug/cmd_vel
/r1/sdk/joint_states
/r1/sdk_transport/motors_healthy
/vr/teleop/active
/r1/safety/kill
```

The default launch is deliberately non-actuating:

```text
transport=mock
send_commands=false
enable_head=false
enable_locomotion=false
enable_prepare=false
commissioning_confirmed=false
profile=slow-safe
```

In that configuration it constructs no Unitree SDK object and publishes only
`/r1/live_writer/debug/*` and status output. Its unit/mock safety work is ready;
it is not evidence of physical R1 commissioning.

The source-backed SDK implementation has these properties:

- Head ownership uses `ArmSdk` on `rt/arm_sdk`, never a partial two-field
  command. It seeds all 13 official ArmSdk fields from fresh feedback before
  the first physical publication and holds/releases safely on stop.
- Locomotion uses only high-level `r1::LocoClient`: gated `StandUp`, bounded
  `SetVelocity`, and `StopMove`. It never uses `rt/lowcmd` or raw leg joints.
- First-live limits are immutable and require profile `slow-safe`: forward
  `0.20 m/s`, lateral `0.12 m/s`, yaw `0.35 rad/s`, with bounded acceleration,
  head angle, head rate, and short velocity duration.
- `/r1/live_writer/prepare`, `/stop`, `/kill`, and `/reset_kill` are explicit
  services. Stop/kill request the central supervisor latch; that software
  latch does not replace a physical E-stop.

### Interlocks enforced by the writer

Every SDK construction and every physical send requires all of the following
process environment values:

```text
ROBOT_DRY_RUN=0
ROBOT_ENABLE_ACTUATION=1
ROBOT_CONFIRM_OFF_CHARGER=1
ROBOT_CONFIRM_CLEAR_AREA=1
ROBOT_CONFIRM_ESTOP_READY=1
ROBOT_CONFIRM_COMMISSIONING=1
ROBOT_COMMISSIONING_TOKEN=<same configured token, at least 16 characters>
ROBOT_VR_SOURCE_IP=<same configured fixed VR source IP>
```

It also requires `transport=sdk`, `send_commands=true`, the relevant feature
flag (`enable_head` or `enable_locomotion`), `enable_prepare=true` and
`require_prepare=true`,
`commissioning_confirmed=true`, a matching ROS commissioning token, a fixed
matching VR source address, `slow-safe`, an approved prepare state, fresh
finite LowState and command data, a fresh held Deadman, and a fresh explicit
`kill=false`. Any timeout, invalid input, deadman release, kill, disconnect,
or shutdown fails closed to stop/hold rather than emitting a new command.

These gates are defense in depth, not authorization. `StandUp` has been
commissioned once; head and locomotion output remain uncommissioned. The
default live ROS domain is
`R1_LIVE_ROS_DOMAIN_ID=88`; the shared domain is intentional so an existing
safety supervisor, reader, stop, and writer can see one another. A caller that
deliberately provides every gate can start the physical-session wrapper, so it
must be treated as a real robot operation rather than a test command.

## Commissioning order

The software stages are implemented and one staged `StandUp` succeeded. Head
and locomotion motion have not been accepted. The following is the required
operator-led order, not a
request to operate the robot now:

1. Robot off its charger, secured/appropriate support, physical E-stop in
   reach, clear workspace, and a spotter present.
2. Read-only LowState and video checks using `make robot-readonly-preflight`;
   verify the command-writer interlock remains closed.
3. Build, unit-test, and run only mock writer checks, including
   `make r1-head-ownership-probe-mock-smoke`. Verify the central software kill,
   stale-data handling, Deadman release, overspeed, and network-loss paths.
4. Correlate a fresh read-only head q/dq sample with the physical pose. Only
   after a separate review may the gated ownership probe test the ArmSdk axis
   with a `≤0.005 rad` out-and-back motion. A probe pass validates only that
   bounded handshake; it does not recenter the head. If the head looks straight
   at `yaw≈1.43`, stop and investigate joint indexing/zero semantics. Never
   bypass the normal tracking seed gate.
5. Design and separately approve any actual centring correction. Only after a
   physically verified centred pose and fresh centred q/dq, start ordinary
   head-only tracking, run the gated prepare service, verify stable `FSM=4`;
   for locomotion also verify `Start()` and stable `FSM=811`, then require the
   post-prepare Deadman release/re-press before the smallest
   yaw/pitch motion. The intermediate neutral-velocity sample is required only
   when locomotion is enabled.
6. Confirm head hold and gradual ArmSdk ownership release on Deadman, kill,
   stale feedback, and shutdown.
7. Test locomotion-only next through high-level `LocoClient`, then confirm
   immediate `StopMove()` on every release/timeout/disconnect.
8. Consider combined head/locomotion only after both isolated stages have a
   signed-off result. Physical arm teleoperation is outside this writer stage.

The operator checklist for these gates is maintained in
[`r1_hardware_commissioning.md`](r1_hardware_commissioning.md). Successful
read-only or mock output, a launch parameter, or an environment variable must
never be treated as a substitute for that physical review.

`make robot-live-arm-check` is a read-only acknowledgement and LowState check;
it launches no writer. `make r1-live-writer-live`, `make head-live-test`,
`make locomotion-live-test`, and `make teleop-live` instead delegate to the
fail-closed physical-session wrapper. With incomplete gates they exit before
writer startup; with complete gates they are capable of physical commands and
must not be run while the robot is charging.

Do not run the SDK examples `r1_arm_sdk_dds_example` or `r1_loco_client` on the
robot as a test: both contain real command calls. Until an operator explicitly
starts the above commissioning, use only build/test, mock, `head-live-dry-arm`,
read-only, and the existing simulation launches.
