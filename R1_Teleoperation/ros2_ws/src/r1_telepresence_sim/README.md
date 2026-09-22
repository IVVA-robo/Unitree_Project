# r1_telepresence_sim

Simulation-only integration for the R1 telepresence prototype. It does not
open a Unitree transport and rejects `hardware_enabled:=true`.

## Full Gazebo scenario

Из корня проекта доступен короткий безопасный wrapper:

```bash
/home/unitree/Unitree_Project/R1_Teleoperation/scripts/r1-sim
```

Одноразовая интеграционная проверка (без Pico и робота):

```bash
/home/unitree/Unitree_Project/R1_Teleoperation/scripts/r1-sim-smoke
```

Она проверяет старт URDF/контроллеров, headset-relative параметр, наличие
ключевых топиков и нулевой `/r1/sim/cmd_vel` при старте.

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws/install/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select r1_kinematics_control r1_telepresence_sim
source install/setup.bash

ros2 launch r1_telepresence_sim telepresence_sim.launch.py \
  display:=false hardware_enabled:=false model_z:=0.25
```

The launch starts Gazebo, the URDF and controllers, the calibrated arm IK,
the deadman-gated `/r1/sim/safe_cmd_vel -> /r1/sim/cmd_vel` simulation adapter, a
simulation-only leg visualizer, the whole-body planner, and the desktop stereo
fallback. Set
`display:=true` when a desktop OpenCV window is available; headless runs should
use `false` and inspect diagnostics instead.

### Stick-driven legs in Gazebo

`r1_sim_leg_visualizer` subscribes to the safe `/r1/sim/cmd_vel` output and the
`/vr/teleop/active` deadman. It applies a conservative alternating leg pose to
the Gazebo-only model and stages it on `/r1/sim/leg_trajectory`, publishing the
requested positions as a `sensor_msgs/JointState` on
`/r1/telepresence/leg_targets`. The
`r1_sim_whole_body_planner` is the only final controller writer: it consumes
that staged leg target and the IK stream
`/r1_kinematics_control/debug/arm_trajectory`, adds a small opposite-phase arm
counter-swing and bounded waist roll/yaw compensation, then publishes the
coordinated leg, arm, and waist trajectories. Its aggregate target is visible
on `/r1/telepresence/whole_body_targets`. Release the deadman or stop sending
sticks and all three groups are rate-limited back to neutral.

This is a visual demonstration aid, not a walking or balance controller. The
current local URDF fixes `pelvis` to `world`, so the body does not translate and
the proxy cannot prove stable biped gait. The waist compensation and arm
counter-swing are timing/visual cues only; real no-fall walking still requires
Unitree's IMU/contact Sport controller and must not be replaced by this node.
For simulation, the leg, arm and waist joints expose position command
interfaces and their controllers are started automatically. The default
`model_z:=0.25` raises the fixed model by 25 cm so the feet and joint motion are
visible above the ground plane; adjust it for a different camera view.
It is never started by the hardware launch and never opens a Unitree transport.

The visual gait uses the URDF's forward-pitch sign, mirrored hip yaw, and a
small outward stance-roll guard. This keeps a forward stick command from
looking like a backwards kick or crossed legs. It is still only a bounded
visual proxy: it does not provide contact feedback or fall protection.

For a manual stick-only check (in a second terminal, while the launch is
running), keep the deadman alive and send a bounded forward command:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws/install/setup.bash
source /home/unitree/Unitree_Project/R1_Teleoperation/ros2_ws/install/setup.bash
ros2 topic pub -r 10 /vr/teleop/active std_msgs/msg/Bool '{data: true}'
```

In another terminal publish `/vr/cmd_vel` (`geometry_msgs/msg/TwistStamped`) at
20 Hz, for example:

```bash
ros2 topic pub -r 20 /vr/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {z: 0.0}}}'
```

Stop both publishers or send `{data: false}` to return legs, arms, and waist
to neutral. The controller state and requested pose are visible on
`/leg_trajectory_controller/controller_state`,
`/r1/telepresence/leg_targets`, and
`/r1/telepresence/whole_body_targets` respectively.
The simulation launch stores body calibration separately at
`~/.ros/r1_telepresence_sim_body_calibration.json`; it never overwrites the
physical-R1 calibration in `~/Unitree_Project/Configs/Calibration/`.

## Camera contract

The current Gazebo camera plugins publish:

```text
/r1/camera/left/left_eye/image_raw
/r1/camera/left/left_eye/camera_info
/r1/camera/right/right_eye/image_raw
/r1/camera/right/right_eye/camera_info
```

The viewer converts common `sensor_msgs/Image` encodings directly with NumPy,
so it does not depend on `cv_bridge` (the latter can fail when its binary was
compiled against NumPy 1.x while the desktop has NumPy 2.x). It publishes FPS,
estimated skipped frames, CameraInfo readiness, and display state on
`/r1/telepresence/diagnostics`.

OpenXR is not installed in this desktop environment. The fallback is therefore
the verified viewer for now; a future OpenXR/WebRTC client can consume the same
ROS camera topics. No download is required for the current simulation path.

## Safety and limits

`/r1/sim/cmd_vel` is only a Gazebo interface here. The local URDF has no confirmed R1
walking/gait controller, so this adapter must not be presented as proof of
biped locomotion. Deadman release, stale velocity input, NaN, startup, and
shutdown all force zero output. A physical R1 requires a separately reviewed
hardware adapter, E-stop procedure, and hardware-only launch. The simulation
launch rejects `ros_localhost_only:=0`, so its controller topics cannot be
exposed through DDS to a physical robot network.

## Tests

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select r1_telepresence_sim r1_kinematics_control \
  --event-handlers console_direct+
```
