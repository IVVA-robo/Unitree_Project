# r1_live_writer

> **Status — 2026-09-17:** the source code and mock/unit safety checks are
> implemented. The staged physical `StandUp` completed and stable FSM `4` was
> confirmed. At the first normal post-prepare head re-arm, fresh feedback reported
> `head_pitch=0.0073 rad` and `head_yaw=1.4299 rad` (numerically `81.9°` in
> joint coordinates). The physical pose/direction and joint-zero correlation
> were not visually confirmed. The writer failed closed **before** `seed_head`
> or any ArmSdk head frame in that first normal-tracking attempt. In a later,
> separately authorized recenter-only session, the writer seeded all 13 fields
> from fresh feedback near `yaw=1.4606`, `pitch=0.0084`, marked ArmSdk claimed
> locally, and queued full-weight frames. The SDK's asynchronous queue provides
> no remote acceptance acknowledgement. Feedback did not follow the small target and the
> `head_recenter_following_timeout` guard stopped the attempt, held the fresh
> feedback pose, released ownership to `weight=0`, and latched kill. The user
> then reported a red chest indicator. Normal tracking and recenter are both
> blocked pending indicator triage and a separate minimal ownership handshake;
> increasing the following timeout is not an accepted workaround.
> Do not start
> `transport=sdk`, enable any live
> feature, or use a manual SDK launch while the robot is charging. The explicit
> live wrapper fails closed unless all commissioning gates are already present;
> physical commissioning still needs a separate operator-led review.

This package is the isolated, fail-closed physical writer boundary for the
Unitree R1. It consumes only the already-shaped ROS debug outputs and fresh
read-only joint feedback:

- head: `/r1_hardware_adapter/debug/head/joint_trajectory`;
- arms: `/r1_kinematics_control/debug/arm_trajectory` when `enable_arms=true`;
- locomotion: `/r1/locomotion_dry_run/debug/cmd_vel`;
- feedback: `/r1/sdk/joint_states`;
- fail-closed motor health: `/r1/sdk_transport/motors_healthy`;
- deadman: `/vr/teleop/active`;
- transient-local kill: `/r1/safety/kill`.
- downstream faults: `/r1/safety/kill_request` (assert-only; the supervisor
  ignores false requests).

The default launch uses `transport=mock`, `send_commands=false`, and disables
head, locomotion, and prepare. It never constructs a Unitree SDK object in that
configuration. Debug output remains available under `/r1/live_writer/debug/*`.
The only supported current execution is the following safe staged sequence:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
make r1-live-writer-build
make r1-live-writer-check
make r1-sdk-abi-probe
make r1-live-writer-mock
make r1-live-writer-mock-smoke
make r1-head-ownership-probe-mock-smoke
make robot-readonly-preflight
make robot-preflight
```

`make r1-offline-commissioning-check` combines the complete workspace build/tests,
ABI probe, normal and ownership-probe MockTransport smoke, VR dry-run, and Gazebo smoke. The complete
aggregate (not only the ABI probe) runs in a verified network namespace that
contains only loopback. It does not run the physical Ethernet preflight and
refuses to start while an SDK process or known vendor command example already
exists.

`robot-stop` and `robot-kill` are operational recovery commands for an
already-running writer, not offline checks. They may invoke physical
`StopMove`/hold/release before asserting the independent software kill.

`r1-live-writer-mock` force-sets every actuation confirmation to its safe
value, selects mock transport, and passes `send_commands:=false`,
`enable_head:=false`, `enable_locomotion:=false`, `enable_prepare:=false`, and
`commissioning_confirmed:=false`. It cannot create an SDK writer. The mock
smoke target runs the isolated `MockTransport` test on a disposable
localhost-only ROS domain and loopback UDP; it does not open the R1 Ethernet
interface or SDK transport.

The SDK implementation uses only official interfaces found in the local
Unitree SDK2 checkout: `r1::LocoClient` for `StandUp`, `Start`, and
`GetFsmId`; the typed `WirelessController_` publisher on
`rt/wirelesscontroller` for the normal legs stage; and R1 `ArmSdk` on
`rt/arm_sdk`. `SetVelocity`/`StopMove` remain available only to the isolated
RPC diagnostic. Legs are never commanded joint-by-joint. `StandUp()` is not
considered a successful prepare by itself:
an async, cancellable worker polls `GetFsmId()` until FSM `4` has been observed
for the required number of consecutive samples. Polling has a bounded timeout;
an RPC error, an unstable FSM, cancellation, or timeout makes prepare fail
closed. The ROS executor is never held for that multi-second confirmation, so
kill, Deadman, state watchdog, and interlock callbacks remain responsive. No
other SDK call is made concurrently with the prepare worker. A stop requested
during prepare is completed after the worker joins; an ambiguously delivered
`StandUp` remains stop-pending until `StopMove` succeeds.

The physical ROS graph is pinned to `rmw_fastrtps_cpp`; Unitree SDK2 loads its
own matched CycloneDDS C/C++ libraries. `make r1-sdk-abi-probe` verifies this
combination in a private network namespace containing only `lo`, initializes
and tears down `ChannelFactory`, `LocoClient`, and `ArmSdk`, and never invokes
a robot command. The physical preflight runs the same probe before it may
start even the read-only robot-facing reader. The writer independently checks
the active RMW and canonical paths of both loaded DDS libraries before
`ChannelFactory::Init`.

The first ArmSdk publication seeds all 13 owned fields from a fresh JointState
in the official order: five left arm joints, five right arm joints, waist yaw,
head pitch, head yaw. Its gains exactly match the vendor R1 example and are not
installed or published during construction. Release keeps publishing a full
13-field hold while linearly ramping ArmSdk weight to zero over approximately
one second. The release loop itself is bounded and makes a final zero-weight
attempt if timing falls behind.
If that last zero-weight frame is queued after a degraded ramp, cleanup is
reported as successful with a warning detail; only failure to queue the final
zero keeps ownership pending. Failed `StopMove` or ownership release remains
marked active, is retried by the watchdog, and blocks reset/prepare. Stop/kill
services return `success=false` while cancellation or cleanup is pending.

Services:

- `/r1/live_writer/prepare` — validates the gates, starts official `StandUp`
  plus bounded stable-FSM polling asynchronously, and returns that the request
  was accepted. Watch `/r1/live_writer/status` for `prepare_result=...`; only
  that later confirmed result starts post-prepare re-arm. It does not
  immediately authorize output;
- `/r1/live_writer/stop` — latched `StopMove` after this client attempted a
  potentially delivered `StandUp` or velocity command, then a 13-field head
  hold and an approximately one-second ArmSdk weight release. The service does
  not report success while cancellation or SDK cleanup is still pending;
- `/r1/live_writer/kill` — stop/release, latch locally, and request the
  central supervisor to assert kill;
- `/r1/live_writer/reset_kill` — clear only after a fresh external kill=false.

Actual SDK construction and every physical send would require all ROS
parameters, fresh runtime inputs, `profile=slow-safe`, a matching commissioning
token and fixed VR source address, plus these process environment values:

```text
ROBOT_DRY_RUN=0
ROBOT_ENABLE_ACTUATION=1
ROBOT_CONFIRM_OFF_CHARGER=1
ROBOT_CONFIRM_CLEAR_AREA=1
ROBOT_CONFIRM_ESTOP_READY=1
ROBOT_CONFIRM_COMMISSIONING=1
ROBOT_COMMISSIONING_TOKEN=<same >=16-character value as the ROS parameter>
ROBOT_VR_SOURCE_IP=<same value as expected_vr_source_ip>
```

Environment values and confirmations are interlocks, not permission to move a
robot. Physical execution still requires the staged commissioning procedure
and a present operator with the hardware E-stop.

## Safety gates

The environment is only one layer. The launch/ROS layer must also select
`transport=sdk`, `send_commands=true`, the applicable feature
(`enable_head` or `enable_locomotion`), `enable_prepare=true`,
`commissioning_confirmed=true`, a matching commissioning token, and the same
fixed RFC1918/private Pico address in `expected_vr_source_ip`. Public,
loopback, link-local, multicast, and malformed addresses fail closed. Defaults
close every one of those locks. The session, reader, safety-supervisor, stop, and kill helpers
share `R1_LIVE_ROS_DOMAIN_ID=88` by default; select another domain only by
setting that exact environment variable consistently before starting a session.

Each send is then re-authorized at runtime. It requires a finite fresh
`/r1/sdk/joint_states`, a fresh `motors_healthy=true` sample derived from all
26 configured LowState `motorstate` fields, a finite fresh feature command, a fresh held
`/vr/teleop/active`, a fresh explicit clear `/r1/safety/kill`, `slow-safe`, and
an approved `prepare` state. A live `send_commands=true` configuration is
rejected unless both `enable_prepare=true` and `require_prepare=true`.
Feedback, motor-health, command, deadman, and kill timeouts are bounded in
configuration and checked again
before crossing the SDK boundary. Loss of any condition fails closed to
debug-zero/stop/hold; a false or stale motor-health sample during prepare or
owned output also latches kill. Mock mode does not require this physical topic.

Successful prepare deliberately leaves output disarmed. Re-arming requires a
new, ordered operator/input sequence:

1. release Deadman;
2. when locomotion is enabled, receive a fresh neutral **processed** velocity
   while Deadman remains released;
3. press Deadman again;
4. receive fresh commands after that new press before any output is allowed.

A press before the required neutral locomotion sample does not arm the writer,
and a non-neutral sample does not satisfy the neutral gate. Head-only mode
skips step 2. When head and locomotion are enabled together, the writer waits
for both a fresh head command and a fresh velocity command after re-arm before
allowing either output; one feature can never start early using stale input.
Releasing Deadman after reaching the ready state disarms the sequence again.
The retained `/r1/live_writer/armed` flag means ready to send now: in addition
to completed re-arm it requires fresh Deadman, joint state, motor health,
kill-clear, and all enabled command streams. It therefore remains false between the new Deadman
press and the first fresh post-press command sample(s).

Arm-only re-arm also has a neutral hold. The first absolute VR arm target after
prepare, STOP, or KILL is captured as the controller's new neutral and is not
sent as a displacement. The writer first publishes the fresh physical feedback
seed, then accepts only deltas from that captured neutral. Those deltas still
pass the immutable `0.25 rad`/`1.0 rad/s` slew limits and the measured-feedback
follow watchdog. This allows a suspended or repositioned controller to be
re-gripped without replaying its old offset; it does not disable automatic
fail-closed stop on stale feedback, Deadman, or kill.

Head control owns all 13 ArmSdk fields, seeds them from fresh feedback before
the first SDK publish, and never emits a two-field head-only command.
The incoming VR command is a headset-relative delta. The writer rate-limits
that delta first, adds it to the measured physical seed, and only then clamps
the absolute target to the immutable Unitree R1 envelope (yaw `±2.0071 rad`,
pitch `±0.6283 rad`). A zero headset delta therefore preserves a nonzero seed;
it can never snap a nonzero measured seed to the small relative limit. Normal tracking
additionally requires a centred, stationary seed (`|yaw|≤0.35`,
`|pitch|≤0.25`, `|dq|≤0.05`) and refuses a wider pose with an explicit
recenter diagnostic.

Full physical recenter is suspended: `/r1/live_writer/recenter_head`,
`make head-recenter-session`, and `make robot-head-recenter` always reject and
cannot authorize output. The replacement head-only ownership micro-probe never
commands SDK zero. It requires `head_recenter_enabled=true` only as the internal
isolated-mode selector, plus `head_ownership_probe_only=true`,
`head_ownership_probe_confirmed=true`, the new process acknowledgement
`ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1`, completed prepare/re-arm, fresh stable
full-body feedback, Deadman, kill-clear, and all normal live gates. The retired
`ROBOT_CONFIRM_HEAD_RECENTER` value is intentionally insufficient.

The probe seeds all 13 ArmSdk fields at `weight=0`, observes the exact seed,
ramps ownership over approximately one second without changing pose, makes one
yaw-only excursion toward zero of at most `0.005 rad` at `0.005 rad/s`, verifies
fresh same-direction feedback while all other fields stay at seed, aborts above
the independent measured-speed ceiling `0.04 rad/s`, returns to the exact seed,
releases to `weight=0`, disarms, and latches kill. Success is
reported only by a fresh `head_ownership_probe=passed terminal=true` status
containing `release_complete=true`, `kill_latched=true`, and the weight-zero
cleanup detail. `make r1-head-ownership-probe-mock-smoke` covers success and
overspeed abort in a loopback-only namespace. The historical full-zero stepping
helper remains unit-tested for offline comparison but has no live entry path.
Physical use still requires a new explicit on-site approval after indicator and
pose/encoder correlation; a probe pass does **not** recenter the head or permit
ordinary tracking.
The normal locomotion stage converts the already bounded physical velocity to
the axes captured from Unitree Explore: forward → `ly`, lateral → `lx`, yaw →
`rx`, with `ry=0` and `keys=0`. It publishes at 20 Hz. On neutral, STOP,
Deadman release, watchdog failure, or teardown it publishes six all-zero
frames over 300 ms. `rt/lowcmd` and raw leg-joint control are not used.

When arms are enabled, the existing headset-relative IK node remains in
`dry_run=true` and publishes ten named arm joints. The writer validates the
complete set, applies a second `0.25 rad` per-command / `1.0 rad/s` slew bound,
merges those targets with a fresh 13-joint feedback seed, and sends the full
frame through the same official `ArmSdk` publisher. Missing or stale arm input
triggers the writer watchdog and releases ownership just like stale head input.

## Stop and kill

`/r1/live_writer/stop` requests a bounded locomotion stop whenever this writer
may have delivered `StandUp` or velocity, a head hold when claimed, and the bounded
approximately one-second ArmSdk release. The potentially-delivered state is
set before calling either high-level RPC: therefore a timeout, exception, or
nonzero result is treated as ambiguous delivery and still causes `StopMove()`
during safe-stop and destruction. The first ArmSdk seed is likewise marked as
potentially delivered before its publisher boundary; if it fails ambiguously,
cleanup sends only a bounded `weight=0` relinquish frame, never a new full-weight
hold. A failed `StopMove()` or ArmSdk release keeps
the output marked active, so the watchdog retries and reset/prepare stay
blocked; only confirmed cleanup clears it. In normal wireless-controller mode,
confirmed cleanup is the bounded six-frame all-zero burst; in the legacy RPC
diagnostic it is `StopMove()`. Stop/kill service responses remain
false while that cleanup or an asynchronous prepare cancellation is pending.
On the R1, `StopMove()` is implemented by the vendor SDK as
`SetVelocity(0, 0, 0)`. Some firmware rejects that RPC in the static standing
FSM 4 (observed return code `127`). The same value was returned by `ai_sport`
after `Start()` reached FSM 811. During a bounded legs-only test the body
shifted but no foot stepped; consequently the ordinary writer treats `127` as
an ambiguous failure. This value
is an in-process RPC result returned by `LocoClient::SetVelocity`; it is not
Linux process exit code 127, and it does not mean that the executable, ROS
package, or a shared library could not be found. A capture of a successful
walk from Unitree Explore established that the app does not call
`SetVelocity` for its virtual stick. It publishes normalized axes on
`rt/wirelesscontroller` at approximately 20 Hz; the robot first shifts its
body/centre of mass and then steps. The ordinary legs stage now reproduces
this typed stream. After each robot boot the operator selects blue **Run**,
then closes the control screen and does not use the phone stick while our
writer is active. A static-only session may therefore confirm
STOP from five consecutive read-only FSM 4 samples, but only when this writer
has never attempted `Start` (FSM 811) or `SetVelocity`. After either locomotion
boundary, a nonzero `StopMove` remains ambiguous and cleanup stays latched.

An independent R1 owner reported that high-level walking works in FSM `811`
even while `SetVelocity()` returns `127`. The separate
`make locomotion-127-probe` experiment can therefore provisionally admit that
status for nonzero velocity only. It requires exact acknowledgements for
`ai_sport 1.0.2.154`, a fully closed Unitree Explore app, and the experiment
itself. The writer forces a forward-only command, caps it at `0.15 m/s`, sends
at `10 Hz`, and ends the interval after `1.5 s`. The general transport still
reports `127` as failure, `StopMove=127` is never accepted as confirmed stop,
and the final velocity lease remains bounded to one second.
`/r1/live_writer/kill` additionally latches locally and asks the central
supervisor to assert `/r1/safety/kill`; `/reset_kill` requires a fresh external
clear state. `make robot-stop` and `make robot-kill` assert the same central
software interlock after first requesting stop/kill from an already-running
writer. They never start a writer or enable motion. None of these is a
replacement for the physical E-stop.

`make robot-live-arm-check` and `make teleop-live-preflight` are read-only
checks: they require all acknowledgements but never start an SDK/DDS writer,
stand, head, or locomotion command. `make r1-live-writer-live`,
`make head-live-test`, `make locomotion-live-test`, and `make teleop-live`
delegate to the explicit fail-closed physical-session wrapper. It returns
`BLOCKED` before writer startup if a gate is missing; with all gates present it
is capable of physical commands and must only be used after the separate
commissioning decision, with the robot off its charger and physical E-stop
coverage.
