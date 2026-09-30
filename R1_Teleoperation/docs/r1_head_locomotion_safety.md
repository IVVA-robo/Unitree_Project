# R1 head and locomotion safety boundary

This document describes the implementation boundary for headset-driven head
tracking and high-level locomotion. The default project mode is always
`ROBOT_DRY_RUN=1`. The source-backed `r1_live_writer` package now exists, but
physical commissioning is still incomplete. A staged `StandUp` reached stable
FSM 4, and a later recenter-only attempt crossed the local ArmSdk publisher
boundary, but head feedback did not follow its small target and the writer
failed closed. Its supported launch for ordinary work remains mock-only. The explicit
live wrappers are fail-closed and exit before starting a writer unless every
commissioning acknowledgement is present; they must not be used while the
robot is on its charger or before a separate commissioning review.

## Verified interfaces

The existing Pico/OpenXR path is:

```text
Pico Unity UDP -> vr_teleop_bridge -> /vr/head/pose, /vr/cmd_vel,
                                      /vr/teleop/active
```

`/vr/head/pose` is a `geometry_msgs/PoseStamped`; it carries the HMD
orientation in `vr_tracking`. `/vr/cmd_vel` is only operator intent, not a
hardware velocity command. The bridge watchdog clears Deadman and publishes a
zero intent after missing packets.

The verified physical R1 LowState map has `head_pitch_joint` at HG IDL slot 29
and `head_yaw_joint` at slot 30. The official SDK has no distinct HeadController
or NeckClient; `r1_live_writer` uses the source-backed
`r1::publisher::ArmSdk` route on `rt/arm_sdk`, whose 13-field ownership includes
both arms, waist yaw, head pitch and head yaw. Before its first physical ArmSdk
publication it seeds all 13 fields from fresh feedback, and it holds all fields
rather than publishing two uninitialised head values. Head roll is unsupported
and remains debug-only. A physical recenter attempt seeded this 13-field path
near `yaw=1.4606`, `pitch=0.0084`; queued frames did not produce following
feedback, so the timeout guard held/released to weight zero and latched kill.
Queueing is only a local async-publisher result, not proof of robot acceptance.
The next physical head step is therefore a bounded ownership micro-probe, not a
repeat of the full recenter trajectory.

With the head workspace subsequently clear, the operator confirmed a
repeatable automatic sideways head turn on every R1 boot. No local writer was
running during the correlated read-only capture: 501 samples held near
`head_yaw=2.0063 rad`, `head_pitch=-0.0037 rad`, with yaw close to the official
positive limit `+2.0071 rad`. This is treated as startup/firmware behaviour,
not VR output. Unitree's official
[R1 bracket-start guide](https://support.unitree.com/home/en/R1_developer/bracket_start)
describes this exact left turn as entering the calibration position. It does
not identify numerical zero as a safe centre. Earlier
`1.43..1.46 rad` samples may have been influenced by suspension carabiners in
the head workspace; obstacles must be removed before power-on and those values
must not seed a centring command.

Application startup is intentionally motionless. The boot calibration pose may
later be followed by an explicit operator-requested centring stage, but never
by an automatic command merely because the Unity/APK or ROS launch started.
That future stage stays physically disabled until the ownership micro-probe is
successful and must retain Deadman, fresh motor/position feedback, bounded
rate/following checks, watchdog stop, weight-zero release, and kill latching.

The R1 built-in Arm Action service exposes a persistent writer endpoint on the
same `rt/arm_sdk` channel. Unitree documents that the endpoint may remain while
the service is idle, but an Arm Action and custom `ArmSdk` control must never
run concurrently (`7400` indicates an occupied channel). Consequently endpoint
discovery alone is neither a conflict nor permission. Before constructing our
writer the gate pins `--expected-writers 1`; directly before the one-shot probe,
after our known local ArmSdk endpoint exists, it pins `--expected-writers 2`.
At both boundaries the read-only gate must
receive an idle `rt/arm/action/state` (`holding=false`, `id=0`, empty `name`)
and no typed `LowCmd` samples. Missing/malformed state, any received command,
or observer failure blocks closed.

The read-only gate now keeps the typed `CreateRecvChannel<LowCmd_>` reader and
queries its CycloneDDS entity without creating a publisher. Exit `0` requires
the exact phase-specific set of one or two matched writers, matching topic/type
metadata, the same complete publication-handle set and cumulative count, plus
at least two churn-free seconds. An unmatched reader, extra/replaced writer,
count disagreement or churn returns exit `15`. This resolves the previous
unmatched-reader ambiguity, but
the action/sample/motor-health/deadman interlocks and fresh explicit physical
authorization remain independently mandatory.

Full physical recenter entry points are now hard-blocked. The replacement probe
uses a dedicated `ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1` acknowledgement (the
retired recenter acknowledgement is insufficient), begins with a passive
`weight=0` seed, ramps ownership at the unchanged seed, commands one yaw-only
out-and-back excursion of at most `0.005 rad`, returns to the exact seed, then
releases to `weight=0` and latches kill. Fresh measured head velocity above
`0.04 rad/s` aborts before another probe frame. A probe pass confirms only this bounded
ownership/axis handshake; it neither establishes joint zero nor authorizes
normal tracking.

For locomotion the reviewed direction uses `r1::LocoClient` only for bounded
`StandUp`/`Start` preparation and stable FSM checks. Normal stick motion uses
the official typed `WirelessController_` channel `rt/wirelesscontroller`, as
captured from Unitree Explore. Do not use `rt/lowcmd`, raw leg trajectories, or
unknown speed-mode values in tests.

## Required environment interlocks

All commands default to dry-run:

```bash
export ROBOT_DRY_RUN=1
export ROBOT_ENABLE_ACTUATION=0
export ROBOT_CONFIRM_OFF_CHARGER=0
export ROBOT_CONFIRM_CLEAR_AREA=0
export ROBOT_CONFIRM_ESTOP_READY=0
export ROBOT_CONFIRM_COMMISSIONING=0
export ROBOT_COMMISSIONING_TOKEN=''
export ROBOT_VR_SOURCE_IP=''
```

The implemented SDK path requires every one of the following values before it
may construct an SDK writer/client or emit a physical command. They are listed
here for auditability, not as an instruction to start live operation; the
live wrapper exits before writer startup when any one is absent.

```bash
export ROBOT_DRY_RUN=0
export ROBOT_ENABLE_ACTUATION=1
export ROBOT_CONFIRM_OFF_CHARGER=1
export ROBOT_CONFIRM_CLEAR_AREA=1
export ROBOT_CONFIRM_ESTOP_READY=1
export ROBOT_CONFIRM_COMMISSIONING=1
export ROBOT_COMMISSIONING_TOKEN='<matching random token, at least 16 characters>'
export ROBOT_VR_SOURCE_IP='<fixed expected Pico IPv4 address>'
```

Those strings are a software acknowledgement only. They do not replace the
physical E-stop, a spotter, a stable support frame, or checking that the robot
is disconnected from its charger.

The launch-side locks are independently closed by default:

```text
transport=mock
send_commands=false
enable_head=false
enable_locomotion=false
enable_prepare=false
commissioning_confirmed=false
```

Even a deliberately configured SDK launch is fail-closed unless the ROS
commissioning token equals the process token, the configured VR source equals
`ROBOT_VR_SOURCE_IP`, the profile is exactly `slow-safe`, and the selected
feature is enabled. Every send is rechecked against fresh finite LowState,
fresh finite command input, fresh active Deadman, and a fresh explicit
`kill=false`; live head/locomotion always require a successful gated `prepare`
state. `send_commands=true` is rejected unless `enable_prepare=true` and
`require_prepare=true`. Timeouts, invalid data, loss of VR, Deadman
release, kill, or shutdown request stop/hold behavior instead of a new command.
An unconfirmed `StopMove` or ArmSdk release remains cleanup-pending: the writer
keeps retrying, blocks reset/prepare, and stop/kill services do not return a
false success. A final queued ArmSdk weight-zero frame counts as released even
if the preceding smooth ramp degraded, and that degradation remains visible in
the status detail.

## Kill and Deadman behavior

`r1_safety_supervisor` publishes `/r1/safety/kill=true` at process startup
with transient-local QoS. Controllers must treat the absence of this topic as
unsafe. The operator can assert it immediately through:

```bash
ros2 service call /r1/safety/emergency_stop std_srvs/srv/Trigger '{}'
```

The reset service is intentionally explicit: passing `true` asserts kill,
while passing `false` requests release. In dry-run it permits debug output;
the implemented writer would still have to satisfy every environment, launch,
and runtime interlock.

```bash
ros2 service call /r1/safety/set_kill std_srvs/srv/SetBool '{data: false}'
```

The locomotion Deadman is `/vr/teleop/active`. On release, stale VR input,
network loss, a kill event, NaN input, or shutdown, the dry-run controller
immediately publishes zero debug velocity and logs the cause. A normal zero
stick intent while Deadman remains held is the path that uses profile-specific
smooth deceleration.
The bridge republishes an unchanged Deadman state at 1 Hz, so the independent
dry-run Deadman timeout is 1.5 seconds; the velocity-command watchdog remains
0.25 seconds. This avoids incorrectly stopping valid held input while still
requiring fresh velocity intent.

`r1_live_writer` adds `/r1/live_writer/stop` and
`/r1/live_writer/kill` services. They request the central software kill and,
if a writer had previously become active, issue the SDK stop/hold/release path.
`make robot-stop` and `make robot-kill` first request those services from an
already-running writer, then assert the central software interlock. They never
start a writer or enable motion. Neither service nor Make target is a
substitute for the physical E-stop.

## Dry-run operation

Build the current workspace and start the fully local diagnostic pipeline:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
cd ..
make teleop-dry-run
```

`r1_teleop_dry_run.launch.py` starts the safety supervisor, optional VR bridge,
head debug controller, and locomotion debug controller. It force-sets every
live environment variable to its safe value and keeps ROS discovery local to
the laptop. `start_arm_pipeline:=false` is the default, so a robot, Ethernet,
and Unitree SDK reader are unnecessary. The optional arm pipeline is
read-only; it must be requested explicitly and still has no command writer.

The bridge can be restricted to a fixed Pico source with
`vr_allowed_source_ip:=<PICO_IP>` (or
`R1_DRY_RUN_VR_ALLOWED_SOURCE_IP=<PICO_IP>` through the wrapper). An empty
source is acceptable for mock/dry-run because the bridge locks its first valid
sender, but a physical commissioning transport must require a verified fixed
source or an authenticated replacement.

The supervisor begins killed. For a diagnostics-only test, release it with:

```bash
ros2 service call /r1/safety/set_kill std_srvs/srv/SetBool '{data: false}'
```

Then, with a fresh HMD pose, save the neutral orientation:

```bash
ros2 service call /r1/head/calibrate_neutral std_srvs/srv/Trigger '{}'
```

The head controller publishes raw, relative, and clamped yaw/pitch/roll plus a
debug-only `JointTrajectory` below `/r1_hardware_adapter/debug/head/`. It has
no subscription or publisher connected to a robot controller. The locomotion
controller publishes only below `/r1/locomotion_dry_run/debug/`. Its selectable
profiles are `slow-safe`, `normal`, and `exhibition`; none is a physical-R1
recommendation.

YAML may lower those bounds but cannot raise the immutable dry-run ceilings:
head target is capped at `0.80 rad`, head angular rate at `2.0 rad/s`, and
locomotion limits/ramp rates at the conservative `exhibition` maximum. A long
executor pause is capped to a `0.10 s` head rate-limit step, preventing a
fresh pose from becoming a target-sized jump.

For an immediate software stop, use:

```bash
ros2 service call /r1/safety/emergency_stop std_srvs/srv/Trigger '{}'
```

The complete mock/replay check needs neither a robot nor a headset:

```bash
make teleop-dry-run-smoke
```

It selects an unused localhost-only ROS domain and UDP port, proves startup
kill, explicit dry-run release, neutral calibration, bounded non-zero debug
head/loco targets, Deadman zero, and stale-packet zero, then terminates only
the processes it created.

The isolated writer can also be built and inspected without a robot, SDK
transport, or motion command:

```bash
make r1-live-writer-build
make r1-live-writer-check
make r1-live-writer-mock
make r1-live-writer-mock-smoke
make r1-head-ownership-probe-mock-smoke
make head-live-dry-arm
```

The mock target force-sets all eight environment values to their safe defaults,
uses `transport=mock`, disables every feature, and emits only
`/r1/live_writer/debug/*` diagnostics. The mock smoke target uses a disposable
localhost-only ROS domain and loopback UDP to exercise prepare, Deadman, kill,
and watchdog behavior through `MockTransport` only. `head-live-dry-arm` starts
the real HMD/head shaping chain but keeps the final writer at
`transport=mock, send_commands=false` and starts no robot reader.

## Live command is fail-closed, not a default mode

`make robot-live-arm-check` and `make teleop-live-preflight` are named
read-only acknowledgement checks: they validate all eight live values, the
link, and fresh LowState feedback but never start a writer or command client.
`make r1-live-writer-live`, `make head-live-test`,
`make locomotion-live-test`, and `make teleop-live` delegate to the explicit
physical-session wrapper. It returns `BLOCKED` without constructing a writer
when any acknowledgement, fixed VR IP, interface/link check, or UDP-port check
is missing.

If all of those gates are deliberately supplied, the latter targets are capable
of starting an SDK session and are therefore **not** dry-run commands. Physical
R1 commissioning, E-stop validation, and the first-motion review are still
open; do not run them on a charging robot or without a separate operator-led
commissioning decision. The common live ROS domain is
`R1_LIVE_ROS_DOMAIN_ID=88` by default.

## Prepare command

```bash
make robot-prepare
```

This command is physical and is valid only while one of the fail-closed live
wrappers is already running in another terminal. It does not start a writer.
It verifies that the existing writer uses `transport=sdk`,
`send_commands=true`, `profile=slow-safe`, and `enable_prepare=true`; then it
runs strict LowState and VR checks and requires a fresh held Deadman. Only
after those checks does it release the central software kill, reset the local
writer latch, and call `/r1/live_writer/prepare`.

The transport sends the official R1 `StandUp()`, polls `GetFsmId()` until
`FSM=4` is stable. In a locomotion-enabled session it then sends the official
`Start()` and requires stable locomotion `FSM=811`, all within bounded
timeouts. A successful Unitree Explore capture then established the missing
command path: the app publishes `WirelessController_` axes on
`rt/wirelesscontroller` at approximately 20 Hz and does not use `SetVelocity`
for its virtual stick. Forward input visibly shifts the body/centre of mass
before the first step. The ordinary isolated legs stage now uses that same
typed channel. The operator selects blue **Run** after boot, closes the control
screen, and confirms that no phone stick will be used concurrently. The
forward axis of this isolated route was physically verified on 2026-09-24:
the robot walked from a short left-stick command, then returned to zero on
neutral/released Deadman and stopped under the normal KILL path. The combined
arms/head/legs path remains blocked until lateral and yaw are verified in the
same isolated manner.
An independent R1 report states that high-level walking can work in FSM `811`
while `SetVelocity()` returns `127`. The new diagnostic therefore admits that
status only in `velocity_status_127_probe_enabled` mode: legs-only,
forward-only, `0.15 m/s` maximum, `10 Hz`, and a `1.5 s` command window. It
requires exact firmware, closed-app, and probe acknowledgements. Every other
nonzero status remains fail-closed, and a `127` result from `StopMove()` remains
ambiguous; the software KILL is latched and the final one-second velocity lease
is allowed to expire.
Head/arms-only sessions deliberately stop at stable
`FSM=4` and never enter sport mode. Command output remains unarmed during
these transitions. The operator must then release Deadman and press it again;
when locomotion is enabled, a fresh neutral processed velocity is additionally
required while Deadman is released. Head-only mode skips that velocity gate.
This post-prepare edge discards pre-stand command samples and prevents a held
stick from moving the robot as soon as StandUp returns.

## Preflight

```bash
make robot-preflight
```

The command is read-only. It checks the selected Ethernet link and ICMP route,
the safety environment, kill state, Deadman, head-pose and locomotion stream
presence/rate, and the selected locomotion mode. `--require-vr` turns a missing
HMD stream into an error. It does not call DDS APIs, `sport`, `ArmSdk`,
stand/stance, or stiffness commands.

When the unified diagnostics tree is running it additionally reports both
controller status messages, their local age, watchdog/Deadman state and the
effective head/locomotion limits. The physical prepare wrapper independently
requires fresh bounded command output for every enabled feature.

It also reports local ROS transport age from the bridge's ROS header timestamp
to the preflight observer. This is intentionally not advertised as end-to-end
HMD latency: the Unity client timestamp is not currently forwarded through the
ROS message contract, and unsynchronised/simulated clocks are marked as
unavailable rather than producing a misleading value.

## Staged workflow

The software/mock sequence that sends no R1 command is:

```bash
make r1-live-writer-build
make r1-live-writer-check
make r1-live-writer-mock
make r1-live-writer-mock-smoke
make r1-head-ownership-probe-mock-smoke
make head-live-dry-arm
make robot-readonly-preflight
make robot-preflight
```

`robot-readonly-preflight` may observe existing R1 LowState telemetry but never
opens a command writer. `robot-live-arm-check` is a later read-only gate and
must only be considered after the robot is physically off its charger and all
eight acknowledgements are honestly set. Do not substitute successful mock
checks, a clear ROS kill, or the software stop target for a physical E-stop and
a reviewed commissioning checklist. Once an operator explicitly starts
physical commissioning, the required order is head-only session → prepare →
post-prepare Deadman release/re-press → stop/kill; locomotion-only is tested
next with the additional neutral-velocity gate, and combined teleoperation
last.
