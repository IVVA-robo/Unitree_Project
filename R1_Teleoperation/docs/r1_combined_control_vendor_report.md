# Draft for Unitree: R1 arm overlay changes FSM811 to 816 and walking stops

Prepared 2026-09-25. This report has not been sent or published.

We need simultaneous VR upper-body control and high-level walking on an R1
with five joints per arm. Installed ai_sport is 1.0.2.154; sport API 1.0.0.0.
The robot is connected directly over Ethernet. There is no competing phone
joystick during the test. Video, LowState feedback, head and arms work.

Observed baseline:

- Official StandUp/Start succeeds (FSM 4 then 811).
- Walking using rt/wirelesscontroller works with ArmSdk disabled; operator
  confirmed physical steps in our isolated locomotion test.
- With upper-body ArmSdk active, RPC GetFsmId 7001 returns 816/status 0. In a
  previous full-control test, a 2.580-second forward input reached vx 0.20 for
  over a second, with no observed steps and no local kill.

On September 25 we compared an opt-in frame builder against the supported A5
motion path in xr_teleoperate 817fb00 (including August 3 fixes afd77d3/845b25a).
It publishes HG LowCmd only to rt/arm_sdk:

- mode_pr uses the documented percentage weight 0..100;
- mode_machine comes from fresh LowState, not a hardcoded value;
- motor mode 1 only for slots 12,13,15..19,22..26,29,30;
- seed positions are measured, bounded, and held for waist; no numeric-zero
  startup homing; arm/head targets retain our existing bounded VR mapping;
- gains match the upstream upper-body gains; dq/tau/reserves are zero;
- CRC is computed for every frame and release, independently checked against
  the installed Python/vendor CRC library;
- 100 Hz is retained from the installed C++ example, rather than Python 250 Hz.

This is a frame-format comparison, not an exact run of the entire Python
application. Startup homing, period and feedforward are not identical.

Timestamped read-only GetFsmId observations (UTC):

| Time | Event/state |
|---|---|
|00:54:02.550|Before candidate upper-body ownership: 811/status 0|
|00:54:53.608|After candidate upper-body ownership: 816/status 0|
|00:55:44.681|After stop/ArmSdk weight release: 811/status 0|

The operator reported no movement in this candidate trial. We do not interpret
RPC 127 as generic success or infer a documented meaning for 816. This matches
the interaction reported in https://github.com/unitreerobotics/xr_teleoperate/issues/319,
which remains open as of our check on September 25.

Could you confirm:

1. Is simultaneous rt/arm_sdk upper-body control and walking supported on
   ai_sport 1.0.2.154 for this R1 configuration?
2. What is the documented meaning of FSM 816, and which documented mode/API
   permits locomotion while the arm overlay owns the upper body?
3. If another firmware build is required, what exact supported version and
   official installation procedure should be used?
4. Are there required ArmSdk header, timing or weight conditions absent from
   the current examples? Please provide a supported minimal combined example.

No guessed FSM transitions, weight-slot bypass, firmware modification or
repeated Start calls were used. Both trials were stopped with zero velocity
and ArmSdk release; no Damping or ZeroTorque was requested.


## Additional source check: Gazebo and forcing 811

Our previous Gazebo demo is not a hardware locomotion controller:
`unitree_r1_description/urdf/r1.urdf` defines a fixed world-to-pelvis joint;
`r1_telepresence_sim/leg_visualizer.py` computes sinusoidal leg targets;
`whole_body_planner.py` merges them with arm IK and visual torso compensation.
It has no IMU/contact balance feedback and does not use ai_sport or FSM811.
Moving these position trajectories to physical legs would replace the vendor
balance controller rather than repair the current overlay interaction.

A second public report, unitree_sdk2_python issue182, describes the same
811-to-816 transition. Its author reports repeated SetFsmId(811) returning1001
while ArmSdk is active. This is external evidence, not a test we repeated or
an official explanation of the state machine. The author's binary-mode_pr
interpretation is a hypothesis and contradicts the percentage convention in
the vendor R1 ArmSdk wrapper; it must not be adopted as a protocol correction.
The issue and September23 comment remain open/unanswered by maintainers at
our check. G1 FSM801 and G1 weight-slot conventions are not R1 solutions.
https://github.com/unitreerobotics/unitree_sdk2_python/issues/182

No R1-specific continuous upper-body RPC alternative was found in the
installed SDK: the documented built-in Arm Action service also uses ArmSdk,
and does not establish arbitrary VR arm control during walking. A supported
combined interface/firmware remains to be identified. No forced state loop,
Gazebo-to-hardware path or further physical command was added during this audit.


## Explicit FSM initiator and combined-message audit

Searched the runtime sources (scripts, exhibition manager, panel, VR bridge,
writer and R1 C++/Python clients). There is no executable SetFsmId(816) call.
The normal prepare path calls StandUp() -> SetFsmId(4), then Start() ->
SetFsmId(811); saved panel Run/Lock commands likewise contain811/4.
816 occurs in diagnostic prose/comments, not as a requested command.
The inspected upstream R1_A5 arm controller publishes rt/arm_sdk and does
not call SetFsmId(816). The observed transition is therefore robot-side;
we cannot identify its exact internal condition without controller source
or a vendor description.

No R1 high-level combined velocity-plus-arm-target message was found in the
installed SDKs. R1 SetVelocity serializes velocity[3] and duration; ArmSdk uses
HG LowCmd (mode_pr, mode_machine, motor_cmd[35], reserve and crc) with no planar
velocity field. Motor dq is individual joint angular velocity, not vx/vy/yaw.
Putting arbitrary q fields into velocity JSON does not establish an API.

The physical full-control tests already use rt/wirelesscontroller for walking,
not merely the problematic SetVelocity RPC. That route walks in the isolated
legs test but yielded no steps with the ArmSdk overlay. Therefore a claim that
only the SetVelocity endpoint's routing is wrong is not established.

A recovery step after a push demonstrates reactive balancing/stepping. It does
not establish identical WBC internals across states, arbitrary arm-motion plus
walking support, or an accessible combined command interface. The earlier
video has no synchronized FSM record proving its mode was816. No pushing test
or state-forcing loop was requested or performed for this audit.

The installed R1 Python client additionally registers sport API7106,
SetTaskId(data), wrapped by WaveHand and ShakeHand with task IDs0..3.
Its request carries only a task ID; no continuous arm target/velocity tuple
is exposed. This is a predefined gesture interface and not a demonstrated
alternative for arbitrary VR arm tracking. It was inspected, not invoked.


## Hybrid High-level legs + direct LowCmd arms audit

No physical LowCmd command was sent. The installed official C++ examples
`example/r1/low_level/r1_ankle_swing_example.cpp` and
`example/r1/low_level/r1A_wrist_swing_example.cpp` both call MotionSwitcher
ReleaseMode until the existing motion mode name is empty BEFORE constructing
the rt/lowcmd publisher. The Python R1 low-level example does the same.
The wrist example addresses only arm/head indices, but still releases the
motion service; it does not demonstrate coexisting with ai_sport walking.

The HG LowCmd carries all35 motor entries. The R1 examples explicitly describe
motor mode1 as Enable and mode0 as Disable. Neither the inspected R1 API nor
MotorCmd schema documents a per-joint ownership/ignore mask that preserves
Sport's leg commands while another writer controls arms. Zero q/kp/kd/tau or
mode0 must not be assumed to mean 'leave ai_sport unchanged'. DDS discovery
or packet delivery cannot establish this arbitration behavior.

Read-only native DDS discovery on enxb4b024be59fe/domain0 ran for8seconds,
with no application writer or motion API calls. Saved:
`dds-hybrid-discovery-20260925.json` and `dds-hybrid-topics-20260925.tsv`.
175 unique direction/topic/type triples were observed. rt/lowcmd already has
publication and subscription endpoints. rt/arm_service_inbound/outbound and
rt/api/arm endpoints also exist. No topic named teleop_cmd or wbc_cmd was
observed in this interval; absence is not proof that no other interface exists.
The arm_service topics are std_msgs String, whose type alone provides no
supported command schema or guarantee of simultaneous walking and arms.

The 'hardcoded safety decision' and detailed listener/WBC architecture in the
external advice remain hypotheses, not established facts about closed firmware.
An ArmSdk publisher endpoint can already exist while the robot remains811;
our evidence concerns active overlay commands/ownership, not mere class init.

Time-multiplexed handoff is a possible separate sequential mode, not simultaneous
control and not automatically safe: velocity must be stopped and handover/state
confirmed before changing upper-body ownership. The user requested simultaneous
control, so no automatic handoff policy was added.

Next prerequisite for the proposed direct-LowCmd hybrid is a vendor-supported
R1 per-joint arbitration/ignore contract with ai_sport active. Without that,
a floor-standing trial risks competing whole-body command streams or disabling
leg output rather than cleanly dividing arm/leg ownership.
