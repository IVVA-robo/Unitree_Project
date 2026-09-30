# R1 A5 ArmSdk packet audit — 2026-09-25

This is a source and offline audit. No robot RPC, writer, motion service, or
physical test was started for it. Simultaneous walking and upper-body control
on the installed `ai_sport 1.0.2.154` remains unverified.

## Finding

The default legacy path in local `SdkTransport::Impl::publish_arm` follows the installed
Unitree C++ ArmSdk example accurately. The newer upstream Python A5 motion
implementation differs in three packet fields and one addressed joint slot.
These differences justify an explicit candidate for comparison; none proves
why this robot changes from FSM 811 to 816 or establishes a firmware fix.

| Property | Local transport | Pinned current A5 motion implementation |
| --- | --- | --- |
| Topic / message | `rt/arm_sdk`, HG `LowCmd_` | Same |
| Blend weight | `mode_pr = int(weight * 100)`, 0–100 | Same convention; initially 100 |
| `mode_machine` | Default 0 | Copied from received LowState at initialization |
| Per-joint `mode` | Default 0 | 1 on addressed upper-body slots |
| Addressed slots | 15–19, 22–26, 13, 29–30 | Same plus slot 12 |
| Motor target | Measured seed, bounded subsequent targets | Measured seed, subsequent bounded arm targets |
| `kp` / `kd` | Arms 50/2, 50/2, 40/2, 40/2, 30/2; yaw 50/3; head 15/1 | Same for shared slots; slot 12 also 50/3 |
| `dq` | 0 | 0 |
| `tau` | 0 | 0 initially, arm feedforward subsequently |
| CRC | Default 0 | Recomputed before each write, including release |
| Typical write period | 10 ms | 4 ms |
| Startup torso/head target | Existing measured seed / local startup procedure | Explicit 3-second interpolation to numeric zero |

`RealTimePublisher` copies `msg_` and calls `Write` without hidden header,
motor mode, or CRC initialization. Its hooks are empty and `ArmSdk` does not
override them. The missing values are therefore real outgoing differences,
not only different source syntax. The installed old C++ ArmSdk example omits
the same values; local behavior was not simply an incorrect copy of it.

The Python subscriber continues to update its `self.mode_machine`, but the
outgoing `self.msg.mode_machine` is assigned only during initialization.
Freshness validation or updating it later would be local design work, not an
exact behavior already provided by that example.

## CRC and layout

Both installed SDKs use polynomial `0x04c11db7`, initial accumulator
`0xffffffff`, processing bit 31 to bit 0 of each word, with no final XOR.
The SDK2py HG command checksum input is an explicitly packed little-endian
record:

```text
<2B2x + (B3x5fI × 35) + 5I
```

It is 1004 bytes: mode_pr and mode_machine plus two zero pad bytes; 35 motor
records of mode plus three zero pad bytes, q/dq/tau/kp/kd floats and reserve;
four message reserve words; then CRC. Checksum covers the first 1000 bytes
(250 little-endian words), excluding the final CRC. This is the vendor's
canonical checksum representation, not a checksum of a DDS packet including
its network/CDR headers.

Prefer explicit packing to `reinterpret_cast<uint32_t *>(&message)` in new
code. Native object padding, aliasing, and host layout should not silently
become checksum inputs. Keep reserved fields and padding deterministic zero.
The HG generated C++ field order agrees with the Python pack order, but that
alone is not a portable native object representation guarantee.

Two offline vectors were independently computed from the packing above and
matched the installed vendor `crc_amd64.so` (no DDS initialization):

| Frame before CRC | CRC |
| --- | --- |
| All fields zero | `0xfe172f9f` |
| Only mode_pr=100, mode_machine=1; all motors/reserves zero | `0x96b2e116` |

Tests should also cover nonzero q/gains, altered header/motor fields, and
release weights, using a structurally independent Python pack/reference.

## Support and evidence boundaries

The saved August 3 commit `afd77d3` removes the broad R1 A5/A7 motion guard,
adds the R1 0–100 weight convention, and restricts the motion packet to its
upper-body slots. A7 retains an explicit rejection. The follow-up `845b25a`
corrects waist yaw to slot 13, includes waist slot 12, and corrects gains.
Pinned `817fb00` still contains this A5 motion path. Thus current upstream
A5 support exists; the July rejection is not a current blanket prohibition.

Slot 12 is marked unused for R1-A5 and waist roll for R1 in the upstream
comment. The local physical map includes waist roll. Adding it changes the
set of actively commanded joints and must use validated measured feedback;
do not infer a universally safe zero target from the A5 name.

The local full session confirms initial FSM 811, later read-only FSM 816,
working head/arms, no active local kill, adequate sustained walking input,
and no observed steps. The same robot walked with the upper-body overlay
disabled. This supports an overlay/locomotion interaction; it does not show
which field, timing rule, or firmware requirement causes it. The existing
recovery record also cites upstream issue 319 reporting a similar transition
on the same ai_sport version, including a September 17 reproduction. Current
source support is not proof of compatibility with this installed firmware.

## Narrow candidate implementation

1. Build a pure, SDK-independent command representation and deterministic
   checksum with offline golden vectors. Build a separate candidate profile;
   preserve the working legacy default until a supervised comparison.
2. Carry measured `mode_machine` with an explicit timestamp/freshness check
   from the existing LowState reader. Do not hardcode the previously observed
   value 1 or parse an incidental status string as a protocol contract.
3. Candidate frames use motor mode 1 only on the supported upper-body slots,
   measured seeds, existing bounded targets/gains, and CRC on every write,
   including hold and release. Legs remain untouched. Slot 12 requires its
   own validated measured seed before enabling the 14-slot candidate.
4. Keep the established mode_pr weight convention. Do not move weight to a
   G1 unused slot, force arbitrary FSMs, or repeatedly reissue Start to fight
   FSM 816. Preserve explicit release and emergency behavior.
5. Keep period, feedforward, and startup homing unchanged initially to avoid
   combining unrelated behavioral changes. Numeric-zero torso/head homing
   and the Python 250 Hz loop are not prerequisites proven by this audit.
6. Observe actual FSM after overlay activation in the next supervised test;
   report 816/unknown truthfully rather than retaining a startup-only
   “walking ready” result. If candidate fails, retain evidence and revert its
   opt-in selection; do not label it a verified walking fix.

## Sources

- Local transport: `ros2_ws/src/r1_live_writer/src/transport.cpp`,
  `SdkTransport::Impl::publish_arm`; `include/r1_live_writer/transport.hpp`.
- Installed C++ SDK root:
  `/home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/unitree_sdk2`.
  Relevant paths: `include/unitree/dds_wrapper/robots/r1/r1_pub.h`,
  `include/unitree/dds_wrapper/common/Publisher.h`,
  `include/unitree/dds_wrapper/common/crc.h`,
  `include/unitree/idl/hg/{LowCmd_,MotorCmd_}.hpp`, and
  `example/r1/high_level/r1_arm_sdk_dds_example.cpp`.
- Installed SDK2py sibling `unitree_sdk2_python/unitree_sdk2py/utils/crc.py`
  and `idl/default.py`: HG pack/checksum and default field values.
- Saved upstream:
  `logs/recovery-20260924-wired/public-upstream/robot_arm-817fb00.py`,
  lines 1693–2028 (`R1_A5_ArmController` and indices); sibling commit JSONs.
- [A5 restored support](https://github.com/unitreerobotics/xr_teleoperate/commit/afd77d365b3c84fa56b1a5c03ab5e709aa1d77b4),
  [waist/gain correction](https://github.com/unitreerobotics/xr_teleoperate/commit/845b25a32f7febedf220e830952a7134897adb9d),
  [pinned controller](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm.py),
  [issue 319](https://github.com/unitreerobotics/xr_teleoperate/issues/319).
- Physical observations: `logs/exhibition/control-20260925-032744.log`
  and `logs/recovery-20260924-wired/README.md`.

## Prepared candidate and validation

The opt-in profile is now implemented as `arm_sdk_frame_profile=a5_20260803`.
The staged wrapper accepts `ROBOT_ARM_SDK_FRAME_PROFILE=a5_20260803`; node,
launches, YAML and wrapper still default to `legacy`. Selection alone does
not authorize commands or start a session. Existing live/session gates apply.

`arm_sdk_frame.cpp` constructs a fresh, upper-body-only HG frame and checksum.
The transport starts its metadata LowState subscriber only at authorized SDK
initialization. It takes the machine mode and waist roll from fresh measured
feedback, checks the local URDF waist range ±0.52 rad, and holds that measured
waist target rather than homing to zero. Positive frames require feedback no
older than 500 ms and the same measured machine mode. A passive seed cannot
claim ownership if the waist has moved more than 0.02 rad before first claim.
The captured metadata remains usable for weight-zero release when feedback
becomes stale; all other output and cleanup rules remain in force.

Offline validation on 2026-09-25:
- Installed writer build completed; colcon writer results: **189 tests,
  0 errors, 0 failures, 0 skipped**.
- Independent watcher and composition-launch pytest run: **29 passed**.
- Frame tests cover vendor golden CRC vectors, upper-body subset, no leg
  commands, weight release, measured metadata, nonfinite/overflow inputs.
- Metadata tests cover freshness, mode changes, stale zero release, and
  preclaim waist displacement. Build/test logs are in the recovery folder.

For supervised comparison, select the profile for one full-control session
and run `scripts/r1-loco-state-watch` concurrently (source ROS/workspace and
`scripts/r1-unitree-sdk-env` first). The watcher requires explicit interface,
duration (at most 120 s) and interval (at least 0.5 s), calls only the existing
read-only probe, and records timestamped JSONL. Failure, pending query and
completion clear the previous FSM. It cannot change modes, start writers,
clear emergency stop, or turn a fresh FSM811 into proof of walking.

Before the supervised test, no candidate frame had been sent. At 03:50 MSK, with all live
writers stopped, six read-only samples returned FSM811/RPC0. This provides
a useful baseline for observing the next ownership transition, not evidence
that the candidate fixes simultaneous control.

## Supervised candidate result — 03:54–03:55 MSK

The candidate was run after the operator returned and confirmed readiness.
Runtime logs confirm `arm_sdk_frame_profile=a5_20260803`, head_claimed=true,
fresh feedback, motor health and VR streams, and no local kill during input.
The operator reported no movement. Read-only captures show FSM811 before
ArmSdk ownership, FSM816 after ownership at00:54:53.608UTC, and FSM811 after
release at00:55:44.681UTC. The packet changes did not prevent this transition.
This is not proof of a universal firmware prohibition; simultaneous control
remains unresolved for this configuration. No additional speculative motion
commands or firmware changes were attempted.

The session `1da6d3f6fae14560b4b7dba194e1fc5f` was stopped with
safe_stop_confirmed=true, all writers exited, and the camera service restored.
Evidence: `control-20260925-035403.log`, `a5-live-fsm-20260925.jsonl`.
A vendor report draft is in `docs/r1_combined_control_vendor_report.md`;
it has not been sent. Defaults remain `legacy`.
