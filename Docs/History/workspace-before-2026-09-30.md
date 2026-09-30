# Unitree R1 workspace

This directory is the canonical home for the Unitree R1 teleoperation,
simulation, VR, SDK, hardware-reference, and Robot POV files on this laptop.

## Layout

```text
Unitree_Project/
├── R1_Teleoperation/       active ROS 2, Gazebo, IK, teleop and Robot POV repo
├── Unity_Projects/
│   └── Unitree_VR_Controller/  Pico/OpenXR controller and native POV app
├── ROS2_WS/                vendor Unitree ROS 2 workspace and SDK checkout
├── SDK/
│   └── Pico/               extracted Pico SDK and its original archive
├── Hardware/
│   └── Photos/             robot, connector and BrainCo hand reference photos
├── Media/
│   └── Robot_POV_Tests/    headset recordings and Robot POV test evidence
├── Tools/
│   ├── Desktop_Shortcuts/  canonical copies of the R1 desktop launchers
│   └── Installers/         project-related offline installers
├── Backups/                small source/config snapshots before risky changes
├── Docs/                   imported notes and cross-project documentation
├── Configs/                shared DDS and saved body-calibration data
├── ROS2_Scripts/           early ROS 2 utilities retained for reference
├── VR_Files/               existing Pico APK artifacts and their index
└── Legacy_Robotics/        older simulation/description workspaces
```

## Canonical paths

- Active project: `/home/unitree/Unitree_Project/R1_Teleoperation`
- Unity project: `/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller`
- Pico SDK: `/home/unitree/Unitree_Project/SDK/Pico/Pico_SDK`
- Unitree ROS workspace: `/home/unitree/Unitree_Project/ROS2_WS/unitree_ws`
- Legacy Gazebo workspace:
  `/home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws`

The old active-project and Pico SDK locations are compatibility symlinks. New
scripts and documentation must use the canonical paths above.

## Where new files belong

| File type | Destination |
| --- | --- |
| Active ROS, Gazebo, IK, teleop or Robot POV code | `R1_Teleoperation/` |
| Unity/Pico source project | `Unity_Projects/Unitree_VR_Controller/` |
| Vendor SDK or ROS workspace | `SDK/` or `ROS2_WS/` |
| Robot and hand photographs | `Hardware/Photos/<date>/` |
| Headset recordings and POV test video | `Media/Robot_POV_Tests/` |
| APK build kept for installation | `VR_Files/` |
| Offline installer | `Tools/Installers/<tool>/` |
| Desktop launcher | `Tools/Desktop_Shortcuts/` |
| Imported note or command cheat sheet | `Docs/Imported_Notes/` |
| Headset/body calibration | `Configs/Calibration/` |
| Pre-change snapshot | `Backups/<component>/` |

The GNOME screenshot library remains in
`/home/unitree/Изображения/Снимки экрана` by user request and is not part of
this consolidation. Desktop entries that point into this project are symlinks;
their real files live under `Unitree_Project`.

## Start here

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
make robot-pov
```

See `R1_Teleoperation/docs/robot_pov.md` for the offline viewer and
`R1_Teleoperation/docs/architecture.md` for the control architecture.

## Safety boundary

Robot POV is video-only by default. The physical R1 adapter is not enabled by
any Robot POV command. Keep real hardware disconnected from command topics
until the hardware interface, E-stop procedure, and explicit hardware launch
have been reviewed.

`R1_Teleoperation/ros2_ws/src/r1_sdk_transport` is the current C++ SDK
read-only boundary. It may read `rt/lf/lowstate` for diagnostics, but it has no
arm or locomotion writer; commissioning interlock and writer flags are
fail-closed.

Run `R1_Teleoperation/scripts/r1-sdk-preflight` for the bounded physical-link,
fresh-26-joint, and safety-parameter checks. It can start the read-only reader
temporarily, but never sends actuator or locomotion commands.
