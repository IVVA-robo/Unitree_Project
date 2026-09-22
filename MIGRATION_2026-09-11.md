# Workspace consolidation — 2026-09-11

The following project-owned files were consolidated into
`/home/unitree/Unitree_Project`. No system packages, ROS user logs, Unity editor
installation, or unrelated downloads were moved.

## Moved directories

- `/home/unitree/Документы/ChatGPT/Unitree r1 Ultra-Lightweight`
  → `/home/unitree/Unitree_Project/R1_Teleoperation`
- `/home/unitree/Загрузки/Pico_SDK`
  → `/home/unitree/Unitree_Project/SDK/Pico/Pico_SDK`

Compatibility symlinks remain at both old locations so running ROS/Gazebo and
older commands continue to resolve while documentation migrates.

## Moved supporting files

- Pico Unity Integration SDK archive → `SDK/Pico/Archives/`
- Robot and hand photographs → `Hardware/Photos/2026-09-11/`
- `IMG_2085.MOV` → `Media/Robot_POV_Tests/`
- NoMachine installer → `Tools/Installers/NoMachine/`
- MuJoCo 3.1.6 installer archive → `Tools/Installers/MuJoCo/`
- Imported R1 simulation context → `Docs/Imported_Notes/`
- Pre-native-POV Unity source snapshot →
  `Backups/Unity/Unitree_VR_Controller.pre-native-pov.2026-09-11/`
- Existing `Unitree_VR.apk` from the desktop → `VR_Files/`
- R1 desktop launchers → `Tools/Desktop_Shortcuts/`
- Legacy command cheat sheet → `Docs/Imported_Notes/unitree_commands.txt`
- Saved body calibration → `Configs/Calibration/r1_body_calibration.json`

The active Unity package reference was rewritten to the canonical Pico SDK
path. Active project documentation now uses
`/home/unitree/Unitree_Project/R1_Teleoperation`.

The three legacy URDF helper scripts and `start_vr_brain.sh` now resolve their
workspace from the script location instead of the removed `~/robotics` path.
Desktop compatibility symlinks were left in place for launchers, the legacy
command note, and `Unitree_VR.apk`.

The IK node, both relevant launch files, package configuration, and operator
documentation now default to the canonical body-calibration path. The former
`~/.ros/r1_body_calibration.json` location is a compatibility symlink.

Both active colcon trees (`R1_Teleoperation/{build,install}` and
`R1_Teleoperation/ros2_ws/{build,install}`) were rebuilt from the canonical
location. Their 116 retained symlink-install entries were atomically retargeted
from the old compatibility path to the canonical source and a subsequent build
confirmed that no old targets returned. The legacy robot-description workspace
and the five-package vendor Unitree workspace were also rebuilt successfully
from their canonical locations. No broken symlinks remain under
`Unitree_Project`.

The active teleoperation suite reports 75 tests with zero errors or failures,
including a regression test that verifies calibration saves do not replace the
compatibility symlink.
The vendor SDK compiles successfully; its optional upstream lint suite remains
non-clean because it checks bundled third-party code and existing vendor
formatting. No vendor source was changed for that lint output.

The 235 files in `/home/unitree/Изображения/Снимки экрана` were deliberately
left where they are at the user's request. System-owned ROS/Gazebo logs,
application caches, the Unity Editor installation, and the extracted MuJoCo
runtime were also left in their standard locations. The old MuJoCo archive path
is a compatibility symlink to its canonical installer copy.
