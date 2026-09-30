# R1 hardware integration readiness

## Current R1 firmware: arm overlay and walking are not jointly verified

On 2026-09-25 the direct main-controller Ethernet path restored real video,
SDK feedback, and user-confirmed walking in the existing **locomotion-only**
stage. Full control successfully moved the head and arms, but walking failed
even with a 2.6-second stick input that reached the configured command ceiling.
Read-only `GetFsmId` returned **816** twice during the full ArmSdk session,
although initial `Start()` had confirmed 811. No local kill was active.

The upstream project initially rejected `--motion` with R1_A5/R1_A7 in
[commit bc55c3f](https://github.com/unitreerobotics/xr_teleoperate/commit/bc55c3f6f68dcc81b343c13a7e083956c538b46d),
but restored R1_A5 support on August 3 in
[commit afd77d3](https://github.com/unitreerobotics/xr_teleoperate/commit/afd77d365b3c84fa56b1a5c03ab5e709aa1d77b4).
The old rejection must not be cited as a current blanket R1_A5 prohibition.
[Issue 319](https://github.com/unitreerobotics/xr_teleoperate/issues/319)
reports the same ArmSdk-triggered 811-to-816 transition on ai_sport 1.0.2.154
and the loss of walking, including physical remote commands. Its September
17 follow-up still reports the problem. Treat walking and upper-body VR as
separately demonstrated capabilities on this robot; combined operation is
unverified. Compare the current supported R1_A5 packet contract before any
new supervised test; do not guess different weights/FSMs or force the FSM
back to 811 repeatedly. Current upstream support does not prove compatibility
with this robot's installed ai_sport build.

The exhibition manager's `ready` state currently confirms that the process
graph passed startup; it does not continuously observe the sport FSM.
`LowState.mode_pr` and `mode_machine` are separate raw values and must not be
used as substitutes for `GetFsmId`. A diagnostic monitor should report the
last successful API 7001 result with its age, distinguish stale/failed reads,
and show a changed FSM without silently calling `Start()` or changing modes.
Even a fresh 811 observation establishes a controller state, not evidence
that the physical robot stepped; combined walking still needs observation.

The separate false arm-response timeout was fixed: a confirmed unchanged
target no longer repeatedly demands movement because of static PD error.
The operator confirmed that raising and holding the right arm continued
working. New-command response checks, joint/rate limits, emergency B and
fresh robot feedback requirements remain in place.

This note records the read-only checks, verified Unitree SDK contract, and the
source-backed `r1_live_writer` safety boundary. A staged physical `StandUp`
confirmed stable FSM `4`. On 2026-09-24 an isolated legs-only session also
confirmed forward walking on the physical R1: after stable FSM `811`, blue
**Run**, and closing Unitree Explore, the writer published the captured
`WirelessController_` contract on `rt/wirelesscontroller`; the operator's
short forward left-stick input produced walking. Returning the stick to neutral
and releasing Deadman produced zero axes, and the normal STOP latched KILL
before the live process was closed. Lateral and yaw motion remain to be tested
separately before any combined upper-body/locomotion session.

Earlier legs-only tests reached stable FSM `811` and crossed several nonzero
`SetVelocity` RPC boundaries. The robot shifted its body but did not take a
step, while `ai_sport` returned status `127`. The retired global compatibility
path must therefore not interpret `127` as accepted velocity during ordinary
operation.
In a follow-up test the Unitree Explore running-person icon was visibly blue,
but the app remained connected; FSM stayed at `811`, the first small
`SetVelocity` again returned `127`, and the robot did not move. Blue **Run**
alone therefore does not resolve the rejection. A retained app-side control
authority is still an unproven possibility and must be isolated by selecting
Run, fully closing/force-stopping Unitree Explore, and only then repeating the
bounded legs-only test.
An independent R1 owner reported in `unitreerobotics/xr_teleoperate#319` that
high-level walking worked in FSM `811` even while `SetVelocity()` returned
`127`, whereas FSM `816` blocked it. This supports one separate experiment in
which only nonzero `SetVelocity=127` is provisionally admitted. It does not
resolve the stop contract because this robot also returns `127` from
`StopMove()`.
It is an undocumented in-process RPC result rather than a Linux process exit
code or proof that `Start()` was omitted. Read-only API probing also found that
the server advertises R1 API `1.0.0.0` but returns `3203` (not implemented) for
both `GetFsmMode` API 7002 and `SetSpeedMode` API 7107. Head output did
not start after LowState reported
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
to zero. It requires publication at **100 Hz** (every 10 ms), including during
the one-second release. `r1_live_writer` preserves this hand-over sequence in its SDK
implementation and must never take control merely because a ROS node started.
Its physical behavior has not yet been validated on the R1.

The same official R1 Arm Control Routine states that `arm_sdk` does not work
in Development/Debugging, because that mode shuts down the built-in controller.
An arm-only test therefore requires the normal Unitree Stand state with balance
running. It does not require `Start()` or locomotion `FSM=811`; enabling sport
to work around an arm problem is outside the arm-only path and introduces leg
authority unnecessarily.

For a physical arm session, the writer also requires meaningful commanded arm
motion to produce fresh same-direction LowState feedback within `0.80 s`.
Otherwise it holds/releases ArmSdk, latches kill, and reports
`arm_feedback_follow_timeout`. This verifies receipt at the robot boundary;
successful local `unlockAndPublish()` alone only confirms handoff to the SDK
publisher thread.

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

The current writer sends `duration=1.0 s` and refreshes the RPC at `10 Hz`, so
the one-second lease is renewed about ten times before expiry. The R1 client
serializes `velocity` and `duration` as JSON; there is no binary velocity IDL
structure whose alignment could explain the observed status. On the tested
firmware both `GetFsmMode()` (API 7002) and `SetSpeedMode(0)` (API 7107) return
`3203` (`API not implemented`), so `SetSpeedMode` cannot be a required working
precondition on this server.

The first retained Unitree Explore screenshot showed **Lock** selected in blue
and **Run** unselected. A follow-up test visibly selected blue **Run**, but the
still-connected app did not change FSM `811` or the `SetVelocity=127` result.
The working legs-only path therefore selects Run, fully closes/force-stops
Unitree Explore, verifies the read-only state, and only then sets
`ROBOT_CONFIRM_RUN_MODE=1`. The dedicated legacy `make locomotion-127-probe` path also
requires `ROBOT_CONFIRM_UNITREE_EXPLORE_CLOSED=1`,
`ROBOT_CONFIRM_AI_SPORT_1_0_2_154=1`, and
`ROBOT_CONFIRM_VELOCITY_127_PROBE=1`. It is legs-only, forward-only, capped at
`0.15 m/s`, refreshes at `10 Hz`, and terminates after `1.5 s`. The ordinary
locomotion path still treats `127` as failure and uses the wireless-controller
transport instead. Combined arm/head/leg live remains disabled until lateral
and yaw tests also confirm the isolated motion and stop path.

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

An isolated or combined locomotion session additionally requires
`ROBOT_CONFIRM_RUN_MODE=1`. The operator-panel checklist sets it after the
operator confirms that Unitree Explore is no longer controlling the robot;
the session then selects the captured official Run/FSM 811 itself.

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
