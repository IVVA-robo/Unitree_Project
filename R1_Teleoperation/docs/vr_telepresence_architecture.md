# R1 VR telepresence: simulation-only architecture

This repository uses one robot-independent intent interface and keeps the real
hardware path disabled.  Gazebo and a future physical R1 can therefore share
the pose, deadman, diagnostics, and trajectory contracts without sharing a
transport implementation.

```text
Pico/Unity/OpenXR
   UDP JSON v1 (poses, HMD, sticks, deadman)
        |
        v
vr_teleop_bridge  -- validation, sequence checks, packet watchdog
  /vr/* topics: PoseStamped, Joy, TwistStamped, Bool
        |
        +--> r1_kinematics_control
        |      HMD-relative body proxy -> EMA/SLERP -> 5-DoF DLS IK
        |      joint limits, rate limit, stale/TF/IK hold
        |      /arm_trajectory_controller/joint_trajectory
        |      /r1/sim/safe_cmd_vel
        |
        +--> r1_sim_locomotion_adapter (hardware_enabled=false)
        |      second watchdog, deadzone/saturation/slew limit
        |      /r1/sim/cmd_vel (Gazebo only; no gait controller is assumed)
        |
        +--> future hardware adapter (separate reviewed package only)

Gazebo R1
  fixed stereo rig -> /r1/camera/{left/left_eye,right/right_eye}/image_raw + camera_info
        |
        +--> r1_stereo_camera_viewer (desktop fallback + diagnostics)
        +--> future OpenXR/WebRTC client (same ROS camera contract)
```

## Coordinate and calibration policy

The Pico publishes HMD and both controllers in the common right-handed
`vr_tracking` frame. Unity `(right, up, forward)` is converted once in the bridge
to ROS REP-103 `(forward, left, up)`; there is no per-hand mirror conversion.

The original IK path then applied two independently captured world-frame neutral
points. Those points were not mirrored and differed in forward/lateral position,
so a room-origin offset appeared as unequal left/right reach. They remain only as
a legacy fallback when `headset_relative_enabled=false`.

The default path now computes the exact HMD-local poses first:

```text
hand_head = inverse(head_world) * hand_world
```

A yaw-only, filtered body frame follows the HMD. Approximate neck, chest, waist,
and shoulder positions come from captured HMD height plus configurable body
ratios. Calibration averages the left pose with a mirrored right pose, producing
exactly symmetric neutral positions while preserving each controller's neutral
orientation. Both arms use one motion scale derived from user and robot arm
reach. User reach, robot reach, behind-shoulder motion, input jumps, Cartesian
target speed, joint velocity, and joint limits are bounded independently.

The identity `torso_link -> vr_tracking` TF in the Gazebo launch is retained for
legacy/debug compatibility. The headset-relative path does not use absolute room
translation as a robot target.

## Единая точка запуска

Для демонстрации все безопасные компоненты собраны в пакет
`r1_teleoperation_app`. Он включает VR UDP bridge, Gazebo/IK/визуализатор ног и
video-only POV-сервер с ROS-источником камер Gazebo. Запуск из корня проекта:

```bash
make r1-teleoperation-build
./scripts/r1-teleoperation
```

Wrapper изолирует DDS (`ROS_LOCALHOST_ONLY=1`, домен 81), проверяет занятые
порты и завершает дочерние процессы одной комбинацией `Ctrl-C`.
`hardware_enabled` принудительно остаётся `false`; hardware adapter и Unitree
моторные API в этот launch не входят. Приложение Unity на Pico остаётся
клиентом трекинга и должно быть открыто отдельно. Состояние можно проверить
без публикации команд:

```bash
./scripts/r1-teleoperation-status
```

## Video decision

OpenXR native runtime is not installed in the current desktop environment, so
the verified implementation is the desktop stereo fallback.  It consumes the
same `sensor_msgs/Image` and `sensor_msgs/CameraInfo` topics that a future native
OpenXR or WebRTC client will use.  In the current URDF these are
`/r1/camera/left/left_eye/{image_raw,camera_info}` and
`/r1/camera/right/right_eye/{image_raw,camera_info}`.  The fallback reports FPS,
estimated skipped-frame counts, CameraInfo availability, and display state on
`/r1/telepresence/diagnostics`.  Raw images are never inserted into the UDP
control packet.

## Locomotion decision

The discovered R1 URDF exposes leg state interfaces but no verified gait command
interface.  The adapter intentionally does not pretend that `/r1/sim/cmd_vel` walks a
biped.  It provides the safe, rate-limited simulation command contract and
publishes zero on startup, deadman release, stale input, NaN, and shutdown.  A
future gait adapter must be a separate package with a confirmed R1 SDK/topic and
an explicit hardware launch.

## Safety invariants

`hardware_enabled` defaults to `false` and the simulation adapter rejects true.
Both bridge and adapter have independent watchdogs.  IK never publishes while
deadman is inactive, holds the last joint state on release, clamps workspace and
joint limits, and rejects stale/invalid poses.  A physical R1 must not be added
to the ROS graph until a reviewed hardware adapter, E-stop procedure, and
hardware-only launch are supplied.
