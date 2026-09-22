# Context Summary: Unitree R1 Simulation & Development on Ubuntu 22.04

## 1. Environment & Tools
* **OS:** Ubuntu 22.04
* **Core Tools:** Python 3.10, MuJoCo (v3.12.0), Pygame, NumPy, Unitree SDK2 (C++ & Python bindings (`unitree_sdk2py`)).
* **Communication:** CycloneDDS (configured via `CYCLONEDDS_URI` and `CYCLONEDDS_NETWORK_INTERFACE=lo`).
* **Simulation Environment:** `unitree_mujoco` (Python bridge variant (`unitree_mujoco.py`)).

## 2. Progress & Architectural Discoveries
* **Model Configuration:** Switched the simulation target from the default quadruped (`go2`) to the humanoid (`r1`) by editing `config.py` (`ROBOT = "r1"`).
* **Physics & Gravity Fixes:** 
  * Enabled virtual suspension bands (`ENABLE_ELASTIC_BAND = True`) in `config.py`.
  * Patched `unitree_mujoco.py` line 24 to attach the elastic band to `"pelvis"` instead of `"base_link"` (which is specific to quadrupeds), successfully keeping the R1 humanoid upright/suspended without collapsing under gravity.
* **Dependencies Resolved:** Installed missing Python modules (`mujoco`, `pygame`) and fixed the `crc_amd64.so` missing shared library error by manually copying the `lib` directory into the user site-packages path for `unitree_sdk2py`.
* **Current Bottleneck:** The standard C++ binary examples (e.g., `r1_arm_sdk_dds_example`, `r1_loco_client`) wait for a full onboard DDS state (`rt/lowstate`), but the lightweight Python MuJoCo simulation bridge does not fully emulate the robot's high-level board computer services, causing network handshakes to hang on `Waiting for connection rt/lowstate`.

## 3. Active Goal for the New Chat
* Transitioning to a compatible control strategy for the R1 humanoid inside the MuJoCo simulation environment or exploring proper Python-based native control scripts that interface directly with `unitree_mujoco` without hanging on missing low-level C++ daemon replies.
```[cite: 1]