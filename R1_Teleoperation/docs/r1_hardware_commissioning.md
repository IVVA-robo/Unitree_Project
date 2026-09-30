# R1 physical commissioning checklist

The physical writer is implemented. After an earlier DDS initialization
failure was contained, a staged `StandUp()` succeeded and stable FSM `4` was
confirmed. The first normal post-prepare head attempt then failed closed before
any ArmSdk frame because fresh feedback showed `head_pitch=0.0073 rad` and
`head_yaw=1.4299 rad`. The yaw value is numerically `81.9°` in joint
coordinates, but the physical pose/direction and joint-zero correlation were
not visually confirmed. In a later separately authorized recenter-only test,
the writer seeded ArmSdk near `yaw=1.4606`, `pitch=0.0084` and queued frames,
but feedback did not follow the deliberately small target. The
`head_recenter_following_timeout` guard held/released ArmSdk to `weight=0` and
latched kill. A red chest indicator was then observed. All writer processes
were stopped. This checklist remains mandatory for head and locomotion
sessions. A passing
mock/read-only test or successful StandUp is not permission to move the head or
walk.

After the head workspace was cleared, the operator confirmed that every normal
boot automatically turns the head sideways. With no local writer running, a
501-sample read-only capture associated that visible boot pose with almost
stationary `head_yaw=2.0063 rad` and `head_pitch=-0.0037 rad`; yaw is close to
the official positive limit `+2.0071 rad`. Treat this as robot startup/firmware
calibration behaviour, not a VR command. Unitree's official
[R1 bracket-start guide](https://support.unitree.com/home/en/R1_developer/bracket_start)
states, “The head rotates to the left into calibration position.” It is not
permission to command numerical zero.
Earlier `1.43..1.46 rad` readings may have been affected by obstructing
carabiners and are not a centre reference.

On 2026-09-24 the operator clarified the repeatable physical sequence: normal
boot turns the head to the **left** calibration pose, while entering either
official Lock or Run brings it visually to the centre. A read-only sample after
the Run transition reported approximately `head_yaw=-0.0002 rad` and
`head_pitch=-0.0004 rad`. On this robot/firmware that observation correlates the
reported `yaw≈0`, `pitch≈0` pose with the visible Lock/Run centre. Startup code
still re-reads fresh feedback after the controller transition and never applies
a fixed offset based only on the boot pose.

## Charging hold

Unitree's official R1 Battery/Charger Manual V1.1 requires the battery to be
powered off and removed from the robot before charging with the official
charger. Let a hot battery cool first, charge it in a ventilated place under
observation, and do not run any robot-facing software while charging.

Do not infer battery health from the programmable RGB strip on the robot's
chest; Unitree exposes that strip through `LedControl`, and no universal
red=fault mapping is documented for it. The four LEDs on the battery are a
different indicator. Normal charging indication there is green; any red
battery LED is a protection/fault indication, so stop charging and follow the
manual rather than continuing commissioning. The official manual is:
<https://marketing.unitree.com/article/en/R1/Battery_Charger.html>.

## Read-only gate

- [ ] R1 control/LAN RJ45 is connected to the selected laptop USB-Ethernet
  interface; carrier is present, the link reports **full duplex**, the receive
  error counter does not grow, and the route to `192.168.123.161` uses it.
- [ ] `make robot-readonly-preflight` reports fresh finite feedback for all 26
  joints, `motors_healthy=true`, `motorstate_nonzero=0`, raw mode bytes,
  fail-closed reader parameters, and no writer process. Any reported
  `slot:code`, false/stale health, or missing health topic is a hard block.
- [ ] `make r1-sdk-abi-probe` passes in its private loopback-only network
  namespace. Confirm it resolves both `libddsc.so.0` and `libddscxx.so.0` from
  the Unitree vendor directory and reports that no command path was used.
- [ ] `make r1-arm-sdk-traffic-check` passes immediately before starting a
  physical writer. One dormant built-in publication is expected and is not a
  pass or fail by itself. Pass only if no typed `LowCmd` sample is received and
  `rt/arm/action/state` is positively observed idle (`holding=false`, `id=0`,
  empty `name`). Missing/malformed state, any action, any command sample, or an
  observer error is `BLOCKED`; do not try to work around Unitree error `7400`.
  This item cannot yet be checked off from exit `0` alone: typed publication
  matching, the stage-specific expected endpoint count, and absence of churn
  remain required read-only evidence before the physical probe.
- [ ] When identifying the active Unitree motion mode is needed, run the
  documented `r1_motion_switcher_probe` once from the laptop. Its SDK Client
  first performs internal `Noop` API `2` (which may be retried) and then one
  explicit `CheckMode` API `1001`; this is outbound diagnostic traffic, not an
  offline test. The probe contains no `SelectMode` (`1002`), `ReleaseMode`
  (`1003`), `SetSilent` (`1004`), publisher, ArmSdk, or LocoClient. It reports
  state only and must never be used as a reason to bypass the normal Stand,
  arm-action, feedback, deadman, or KILL gates.
- [ ] The charging state is checked physically. The verified R1 LowState
  contract has no charging field, so software cannot infer this safely.
- [ ] The existing simulation in ROS domain 81 / UDP 9090 is left untouched;
  choose a separate live port or stop it only as a deliberate operator action.

### TP-Link UE200 revision `2357:0602`

Linux 6.8 does not include this exact USB ID in `r8152`, so the adapter falls
back to CDC configuration 2 and `cdc_ether`. On the development laptop this
caused hundreds of thousands of `rx_length_errors` per second and made R1 DDS
feedback unusable. Linux upstream added this ID in commit `dc9c67820f81`.

The preferred fix is a kernel-supported USB 3 Gigabit adapter (for example a
known `r8152`/RTL8153 device). For a bounded maintenance test with the existing
UE200, first stop every writer and **physically unplug the robot RJ45**, then:

```bash
sudo ./scripts/r1-ue200-driver-test --apply
```

Only reconnect RJ45 after the helper reports configuration 1 with driver
`r8152`. The change is temporary. With RJ45 unplugged, rollback is:

```bash
sudo ./scripts/r1-ue200-driver-test --rollback
```

Rebooting or removing USB is an additional recovery path; no driver blacklist
or boot configuration is installed.

## Before prepare or motion

- [ ] Charger is disconnected and R1 is on a level floor or the approved
  commissioning support.
- [ ] The complete automatic boot sweep of the head has already finished, and
  no carabiner, cable, support, person, or tool can enter the head workspace.
- [ ] Test area is clear; cables cannot enter the legs/head workspace.
- [ ] A spotter is present with immediate access to the physical E-stop.
- [ ] Pico uses a fixed known RFC1918 IPv4 address and its app sends to the
  selected live UDP port.
- [ ] `ROBOT_DRY_RUN=0`, actuation/off-charger/clear-area/E-stop/commissioning
  acknowledgements, a unique 16+ character token, and the fixed Pico IP are
  deliberately exported in every live terminal.
- [ ] `make robot-live-arm-check` passes. This command is read-only and starts
  no writer.

## Arm-only first motion

This stage uses `make arms-live` only. It must not be combined with
`make legs-live` or `make teleop-live`; arm control uses the official
`rt/arm_sdk` interface while locomotion remains disabled.

Start `make arms-live` first. Verify its startup message says
`enable_arms=true`, `enable_locomotion=false`, and `publish_rate_hz=100`;
the graph must remain under the asserted software KILL. With that graph
running, and before arming Deadman, hold both VR controllers in the intended
neutral pose and refresh the headset-relative calibration while Deadman is
released:

```bash
make arms-calibrate
```

`arms-calibrate` joins only the laptop-local ROS graph, verifies a fresh
Deadman=false sample, and calls `/vr/calibrate_body`. It neither constructs a
Unitree SDK client nor clears KILL. The checked-in/current JSON may contain a
synthetic test neutral, so a successful real calibration is mandatory before
the first physical arm prepare.

The kinematics node now checks that the first synchronized pose after every
Deadman re-arm is within `body_proxy.neutral_pose_max_error_m` (default
`0.15 m`) of the saved calibration. A mismatch is held locally and logged as
`body_calibration_neutral_mismatch`; no arm target is forwarded to the live
writer. This prevents a stale left/right controller offset from being turned
into a reach-limited IK target and triggering a physical watchdog stop.

If the installed R1 firmware accepts ArmSdk only while its running controller
is active, use the separately reviewed `make arms-running-live` stage. It keeps
`enable_locomotion=false` and does not start the joystick/`SetVelocity` path,
but `robot-prepare` requests `StandUp()` followed by `Start()` and waits for
stable `FSM=811`. STOP/KILL still call the bounded `StopMove()` cleanup path.
The offline contract is covered by `MockTransport` tests; this mode remains a
physical commissioning action and must not be run while the robot is charging.

- [ ] In the official Unitree application, leave Development/Debugging and
  confirm the robot is in normal **Stand** with its built-in balance controller
  running. `GetFsmMode` error `3203` means the firmware does not implement that
  query; it is not evidence of Debug. Do not use `Start()`/sport or
  `motion_switcher.ReleaseMode()` as an arm workaround.
- [ ] Confirm no Arm Action is actively running. The persistent action service
  endpoint is normal; an active action (`holding=true`, nonzero `id`, or a
  non-empty `name`) is a hard block.
- [ ] Confirm the fail-closed `make arms-live` graph and the successful real
  `make arms-calibrate` result are still visible before prepare.
- [ ] Hold Deadman and run `make robot-prepare`. After stable `FSM=4`, release
  Deadman and press it again. Use only slow, small controller movement for the
  first target.
- [ ] Observe the first arm separately, then the other. A lack of measured
  same-direction joint feedback for `0.80 s` must produce
  `arm_feedback_follow_timeout`, a weight-zero release, and kill. Do not retry
  by bypassing the watchdog.
- [ ] Release Deadman and run `make robot-stop`; confirm neither leg receives
  a command. `make robot-kill` remains available at every point. STOP/KILL
  assert central KILL before waiting for writer cleanup. After either command,
  close the old live launch and start a fresh `make arms-live`; do not reuse its
  persistent SDK writer for another prepare.

## Head-only first motion

Current hold: do not repeat ordinary head tracking or any physical head probe
until the head/controller ownership and visual-centre semantics are resolved.
The physical head/encoder correlation is known for the left automatic boot pose
(`yaw≈+2.0063 rad`) and the visually centred Lock/Run pose (`yaw≈0 rad`).
`make robot-prepare`
performs a five-sample q/dq check before releasing kill and, in normal tracking,
blocks outside `|yaw|≤0.35`, `|pitch|≤0.25`, or `|dq|≤0.05`. Full physical
recenter is now unconditionally suspended; its legacy scripts return `BLOCKED`
without launching a writer or calling a service. Do not increase the old
following timeout or repeat the trajectory. The replacement ownership
micro-probe is implemented and mock-tested, but it still requires the robot to
be healthy, off the charger, and a new explicit on-site approval. Do not
command zero or infer a centre from either the old `1.43..1.46 rad` samples or
the new boot pose; investigate the official startup/parking action and
joint-zero semantics first.

For the FSM 811 investigation, `make head-ownership-probe-session` now prepares
the official running controller while keeping arms, joystick input and the
`SetVelocity` pipeline absent. The micro-probe accepts an encoder seed near
zero without interpreting it as visual centre: it chooses the direction with
more room inside the immutable yaw envelope, moves at most `0.005 rad`, verifies
same-direction feedback, returns to the exact seed, releases ArmSdk and latches
KILL. Only after that probe passes may `make head-running-live` be considered
for a separately observed HMD/head test.

Opening the VR/teleoperation application must never move the physical head.
The intended operator flow is passive startup followed by a separate,
deliberate **Center head** action. That action remains unimplemented/blocked
for physical use until the ownership micro-probe below passes. A future centre
request must require held Deadman, fresh joint and motor-health telemetry, all
live acknowledgements, a clear workspace, slow bounded motion with following
feedback, and a guaranteed hold/weight-zero/kill abort path. Probe success is
still not permission to run that centre request automatically.

### Conditional ownership micro-probe (not recenter)

This is not a command to run now or while the robot is charging. It is the
review checklist for a later, separately approved test after the indicator and
physical pose/encoder correlation are resolved. The probe performs only a
`≤0.005 rad` yaw out-and-back around the measured seed; it never centres the
head and aborts if measured head speed exceeds `0.04 rad/s`. This dedicated
limit was tightened after a 501-sample read-only baseline reported yaw
`dq max=0.02290078 rad/s` with no sample above `0.03 rad/s`; the normal seed
threshold remains `0.05 rad/s`. The old
`make head-recenter-session` and `make robot-head-recenter`
commands remain present only to fail closed.

- [ ] Run `make r1-head-ownership-probe-mock-smoke`; it must pass in its verified
  loopback-only namespace, including the `0.041 rad/s` overspeed negative test.
- [ ] Repeat `make r1-arm-sdk-traffic-check` immediately before the one-shot
  service request. A result from an earlier preflight is too old to authorize
  the probe; any non-zero result keeps both kills asserted.
- [ ] Confirm visually how physical yaw correlates with fresh yaw/pitch q/dq;
  record the seed and ensure a `0.005 rad` motion toward numeric zero is safe.
- [ ] In addition to every generic live acknowledgement, explicitly set
  `ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1` for this one session. The retired
  `ROBOT_CONFIRM_HEAD_RECENTER` acknowledgement is not accepted.
- [ ] Start `make head-ownership-probe-session`. Confirm startup remains killed and
  that locomotion is disabled.
- [ ] Calibrate the HMD neutral, hold Deadman, and run `make robot-prepare`.
  Verify stable `FSM=4`, then release and press Deadman again as instructed.
- [ ] Run `make robot-head-ownership-probe` once and keep Deadman held. Do not
  steer the HMD during this one-shot diagnostic.
- [ ] Accept success only when the fresh terminal status says
  `head_ownership_probe=passed terminal=true`, `release_complete=true`, and
  `kill_latched=true`, includes a successful `weight=0` release, and fresh
  topics confirm `armed=false` plus central `kill=true`.
- [ ] Stop the probe session and verify the head returned to the original,
  stationary seed q/dq. A pass does not authorize normal tracking or an actual
  centring trajectory; review those as separate stages.

- [ ] Start `make head-live-test` in terminal A. Confirm its startup message
  says kill is asserted; merely starting the graph must not initialize SDK
  command transport.
- [ ] While Deadman is released, look straight ahead and call
  `/r1/head/calibrate_neutral`; confirm the head status says `calibrated=True`.
- [ ] Hold Deadman only for the explicit prepare step and run
  `make robot-prepare` in terminal B.
- [ ] Verify `StandUp()` returned success and `GetFsmId()` was stably `4`.
- [ ] Release Deadman, keep controls neutral, then press Deadman again. The
  writer must not accept any command sampled before this edge.
- [ ] Apply the smallest yaw/pitch motion. Roll must remain unsupported.
- [ ] Release Deadman and verify hold plus gradual ArmSdk ownership release.
- [ ] Run `make robot-stop`; separately verify the physical E-stop procedure.

## Locomotion second

- [x] Stop the head/arm session. In Unitree Explore select **Run** and wait until
  the running-person icon is blue; confirm that **Lock** is not selected. Blue
  Run was confirmed, but with the app still connected the first small
  `SetVelocity` returned `127` and the robot did not move.
- [ ] Repeat the isolation with one added step: after selecting Run, fully
  close/force-stop Unitree Explore. Then verify the read-only FSM before any
  writer starts. This tests whether the vendor app retained control authority.
- [ ] Confirm the reported firmware is exactly `ai_sport 1.0.2.154`. Export
  `ROBOT_CONFIRM_RUN_MODE=1`, `ROBOT_CONFIRM_UNITREE_EXPLORE_CLOSED=1`,
  `ROBOT_CONFIRM_AI_SPORT_1_0_2_154=1`, and
  `ROBOT_CONFIRM_VELOCITY_127_PROBE=1`, then start
  `make locomotion-127-probe`. The ordinary `make locomotion-live-test` does
  not admit status `127`.
- [ ] Repeat prepare and the release/neutral/re-press sequence; the first
  accepted locomotion command must be zero.
- [x] Verify prepare also calls `Start()` and reaches stable `FSM=811` before
  any `SetVelocity()` call. The tested firmware returned undocumented RPC
  status `127`; this is not Linux exit code 127.
- [x] Apply one very small `slow-safe` stick command with a spotter. The body
  shifted but no foot stepped because the first `0.0025 m/s` packet returned
  `127` and the ordinary writer stopped immediately.
- [ ] In the dedicated probe, press the stick straight forward and keep it
  steady. The writer forces lateral/yaw to zero, ramps toward at most
  `0.15 m/s`, refreshes `SetVelocity` at `10 Hz`, and latches KILL after the
  `1.5 s` test window. Do not start a second interval in the same session.
- [ ] Verify the robot is physically stationary after the final one-second
  command lease expires. A `StopMove=127` response is not proof of stop.
- [ ] Verify official `StopMove()` on Deadman release, stale VR packets,
  software kill, and session shutdown before increasing duration or speed.
- [ ] Never publish raw leg joint commands or `rt/lowcmd`; only the official
  high-level R1 `LocoClient` is permitted.

## Combined session last

- [ ] Head-only and locomotion-only logs are reviewed and both stop paths pass.
- [ ] Keep `make teleop-live` blocked until the isolated Run-mode test produces
  a real step and the resulting FSM/stop logs are reviewed. Do not combine arms,
  head, and legs in the next physical experiment.
- [ ] `make robot-kill` is tested as a software layer; it never replaces the
  physical E-stop.

The physical commands above require a new explicit on-site decision. Until
that point use only:

```bash
make r1-live-writer-build
make r1-live-writer-check
make r1-sdk-abi-probe
make r1-live-writer-mock-smoke
make r1-head-ownership-probe-mock-smoke
make head-live-dry-arm
make robot-readonly-preflight
```

The equivalent single software-only gate before the next on-site review is:

```bash
make r1-offline-commissioning-check
```

It runs the complete build/test/smoke aggregate in a verified loopback-only
network namespace, does not run the Ethernet preflight or physical reader, and
refuses to start while an SDK/robot-facing process or known Unitree command
example already exists. It also forces both SDK-linked packages to the same
`R1_UNITREE_SDK_ROOT` during incremental builds and audits their CMake cache,
installed launch, RUNPATH, and pre-init ABI guard so an obsolete cache cannot
silently restore a different DDS runtime.
