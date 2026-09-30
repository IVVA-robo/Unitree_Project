R1_UNITREE_SDK_ROOT ?= /home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/unitree_sdk2

.PHONY: robot-pov robot-pov-mock robot-pov-exhibition robot-pov-bad-wifi \
	robot-pov-r1 robot-pov-r1-preflight robot-pov-preflight robot-pov-deps \
	robot-pov-deps-offline robot-pov-cache-deps robot-pov-build \
	robot-pov-check r1-camera-preflight unity-telepresence-validate \
	unity-telepresence-build r1-sim r1-sim-smoke r1-teleoperation \
	r1-teleoperation-build r1-teleoperation-check r1-teleoperation-status \
	r1-sdk-build r1-sdk-readonly r1-arm-sdk-traffic-check \
	r1-sdk-abi-probe r1-sdk-preflight r1-live-writer-build r1-live-writer-check \
	r1-live-writer-mock r1-live-writer-mock-smoke r1-head-recenter-mock-smoke \
	r1-head-ownership-probe-mock-smoke \
	r1-offline-commissioning-check \
	r1-lan-preflight r1-offline-session r1-offline-service-install \
	r1-live-writer-live robot-preflight robot-prepare \
	safety-supervisor \
	robot-readonly-preflight robot-live-arm-check teleop-live-preflight \
	head-dry-run locomotion-dry-run arms-dry-run legs-dry-run head-live-dry-arm head-live \
	head-live-test locomotion-live-test \
	locomotion-127-probe \
	arms-live arms-running-live arms-calibrate head-calibrate legs-live \
	head-recenter-session robot-head-recenter \
	head-ownership-probe-session robot-head-ownership-probe \
	teleop-dry-run teleop-dry-run-smoke robot-stop robot-kill robot-zero-torque teleop-live \
	pov-demo pov-vr pov-stereo-demo pov-high-quality pov-low-latency \
	pov-exhibition pov-preflight operator-panel operator-panel-smoke \
	operator-panel-check r1-panel-status r1-voice-preflight r1-operator-panel-autostart-install \
	exhibition-panel exhibition-static exhibition-control exhibition-reconnect \
	exhibition-lock exhibition-rearm \
	exhibition-calibrate exhibition-calibrate-arms exhibition-calibrate-head \
	exhibition-full-live exhibition-stop \
	exhibition-status exhibition-mock-smoke r1-battery-monitor r1-connection-ensure \
	r1-app-command-capture r1-voice-ensure


r1-sim:
	./scripts/r1-sim

r1-sim-smoke:
	./scripts/r1-sim-smoke

r1-teleoperation:
	./scripts/r1-teleoperation

r1-teleoperation-build:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install --packages-select r1_sdk_transport r1_live_writer --cmake-args -DUNITREE_SDK_ROOT="$(R1_UNITREE_SDK_ROOT)" && colcon build --symlink-install --packages-up-to r1_teleoperation_app'

r1-teleoperation-check:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon test --packages-up-to r1_teleoperation_app && colcon test-result --verbose'

r1-teleoperation-status:
	./scripts/r1-teleoperation-status

r1-sdk-build:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install --packages-select r1_sdk_transport --cmake-args -DUNITREE_SDK_ROOT="$(R1_UNITREE_SDK_ROOT)"'

# Read-only by construction.  Set sdk_enabled=true only after the robot
# Ethernet link is already connected and the operator wants LowState telemetry.
r1-sdk-readonly:
	./scripts/r1-sdk-readonly

r1-battery-monitor:
	./scripts/r1-battery-monitor

r1-connection-ensure:
	./scripts/r1-connection-ensure

r1-app-command-capture:
	./scripts/r1-app-command-capture $${R1_CAPTURE_ARGS:-}

r1-voice-ensure:
	./scripts/r1-voice-ensure

# Bounded read-only LowCmd plus arm-action-state subscribers. Exit 0 requires
# an idle action state and no commands; every other code blocks another writer.
r1-arm-sdk-traffic-check:
	./scripts/r1-arm-sdk-traffic-check

# Inert loader/init/teardown diagnostic. The installed wrapper creates a
# private network namespace containing only loopback before loading SDK2.
r1-sdk-abi-probe:
	bash -c 'set -eo pipefail; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; set -u; ros2 run r1_live_writer r1-sdk-abi-probe'

# Read-only physical commissioning checks. This may start bounded LowCmd,
# action-state, and LowState readers, but never an ArmSdk/LocoClient writer.
r1-sdk-preflight:
	./scripts/r1-sdk-preflight

# The writer defaults to mock/dry-run. Build, check, and mock targets never
# select SDK transport; live targets delegate to a fail-closed wrapper which
# requires every commissioning acknowledgement before it can launch anything.
r1-live-writer-build:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install --packages-select r1_live_writer --cmake-args -DUNITREE_SDK_ROOT="$(R1_UNITREE_SDK_ROOT)"'

r1-live-writer-check:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon test --packages-select r1_live_writer && colcon test-result --verbose'

r1-live-writer-mock:
	bash -c 'set -eo pipefail; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; set -u; ROS_DOMAIN_ID=$${R1_LIVE_ROS_DOMAIN_ID:-88} ROS_LOCALHOST_ONLY=1 ROBOT_DRY_RUN=1 ROBOT_ENABLE_ACTUATION=0 ROBOT_CONFIRM_OFF_CHARGER=0 ROBOT_CONFIRM_CLEAR_AREA=0 ROBOT_CONFIRM_ESTOP_READY=0 ROBOT_CONFIRM_COMMISSIONING=0 ROBOT_COMMISSIONING_TOKEN= ROBOT_VR_SOURCE_IP= ros2 launch r1_live_writer r1_live_writer.launch.py transport:=mock send_commands:=false enable_head:=false enable_locomotion:=false enable_prepare:=false commissioning_confirmed:=false'

r1-live-writer-mock-smoke:
	./scripts/r1-live-writer-mock-smoke

r1-head-recenter-mock-smoke:
	./scripts/r1-head-recenter-mock-smoke

# Canonical name. The older target remains as a software-only compatibility
# alias; both execute the same loopback-only ownership-probe smoke.
r1-head-ownership-probe-mock-smoke:
	./scripts/r1-head-ownership-probe-mock-smoke

r1-offline-commissioning-check:
	./scripts/r1-offline-commissioning-check

r1-lan-preflight:
	./scripts/r1-lan-preflight

r1-offline-session:
	./scripts/r1-offline-session $${R1_OFFLINE_MODE:-r1}

r1-offline-service-install:
	install -Dm0644 systemd/user/r1-offline-session.service \
		"$${XDG_CONFIG_HOME:-$${HOME}/.config}/systemd/user/r1-offline-session.service"
	systemctl --user daemon-reload
	systemctl --user enable --now r1-offline-session.service

r1-live-writer-live:
	./scripts/r1-teleop-live

# `robot-preflight` is a read-only human-readable report and preserves a WARN
# exit as success. `robot-prepare` is different: it can call the official
# StandUp service, but only inside an existing fully acknowledged live session.
robot-preflight:
	@./scripts/r1-teleop-preflight || { code=$$?; [ $$code -eq 2 ] || exit $$code; }

robot-prepare:
	./scripts/r1-robot-prepare

safety-supervisor:
	./scripts/r1-safety-supervisor

# Staged physical-R1 review commands. The first observes only LowState. The
# acknowledgement check is also read-only; it never starts a writer. A later
# live session is fail-closed in its wrapper and must not be run on a charger.
robot-readonly-preflight:
	./scripts/r1-robot-readonly-preflight

robot-live-arm-check:
	./scripts/r1-robot-live-arm-check

# Compatibility spelling for the same fail-closed, read-only acknowledgement check.
teleop-live-preflight:
	./scripts/r1-robot-live-arm-check

# These three targets are diagnostics-only.  They force dry-run environment
# flags and publish exclusively to named debug topics.
head-dry-run:
	R1_DRY_RUN_UDP_PORT="$${R1_DRY_RUN_UDP_PORT:-19130}" \
	R1_DRY_RUN_DISCOVERY_PORT="$${R1_DRY_RUN_DISCOVERY_PORT:-19131}" \
	R1_DRY_RUN_START_LOCOMOTION=false ./scripts/r1-teleop-dry-run

locomotion-dry-run:
	R1_DRY_RUN_UDP_PORT="$${R1_DRY_RUN_UDP_PORT:-19132}" \
	R1_DRY_RUN_DISCOVERY_PORT="$${R1_DRY_RUN_DISCOVERY_PORT:-19133}" \
	R1_DRY_RUN_START_HEAD=false ./scripts/r1-teleop-dry-run

# Explicit arm-only dry-run.  This starts the VR bridge, headset-relative IK,
# and MockTransport writer on localhost; it never opens Unitree Ethernet/DDS.
arms-dry-run:
	./scripts/r1-arms-dry-run

# Explicit locomotion-only dry-run.  The head and arm paths remain disabled.
legs-dry-run:
	R1_DRY_RUN_UDP_PORT="$${R1_DRY_RUN_UDP_PORT:-19132}" \
	R1_DRY_RUN_DISCOVERY_PORT="$${R1_DRY_RUN_DISCOVERY_PORT:-19133}" \
	R1_DRY_RUN_START_HEAD=false R1_DRY_RUN_START_ARM_PIPELINE=false ./scripts/r1-teleop-dry-run

# Full VR head pipeline into MockTransport with send_commands=false. This is
# the explicit head-live-dry-arm mode; it does not open Unitree Ethernet/DDS.
head-live-dry-arm:
	./scripts/r1-head-live-dry-arm

# Physical head entry point.  The wrapper is fail-closed and requires the
# reviewed commissioning acknowledgements; it is never a dry-run alias.
head-live:
	./scripts/r1-head-live-test

# These are explicit, fail-closed physical session wrappers. They require all
# live acknowledgements, a fixed VR IP, link checks, and a fresh-start kill
# latch; they are not dry-run diagnostics and must not be used on a charger.
head-live-test:
	./scripts/r1-head-live-test

# Head-only commissioning with the official FSM 811 controller active. The
# joystick/SetVelocity path and both arms remain absent.
head-running-live:
	./scripts/r1-head-running-live-test

# Legacy full physical recenter entry points are intentionally retained only to
# fail closed with a migration message. They cannot start a writer or request a
# service.
head-recenter-session:
	./scripts/r1-head-recenter-session

robot-head-recenter:
	./scripts/r1-robot-head-recenter

# Separate opt-in ownership micro-probe. It does not recenter the head. Starting
# the session remains killed; robot-prepare and the post-prepare release/re-press
# still happen before the one-shot probe service can be requested.
head-ownership-probe-session:
	./scripts/r1-head-ownership-probe-session

robot-head-ownership-probe:
	./scripts/r1-robot-head-ownership-probe

locomotion-live-test:
	./scripts/r1-locomotion-live-test

# One-shot compatibility experiment for ai_sport 1.0.2.154. Status 127 is
# admitted only for nonzero SetVelocity during a forward-only 1.5 s window.
locomotion-127-probe:
	./scripts/r1-locomotion-127-probe

# Explicit staged aliases requested for the physical commissioning workflow.
# Both wrappers remain fail-closed and require the same acknowledgements as
# the canonical head/locomotion entry points.
arms-live:
	./scripts/r1-arms-live-test

# Arm commissioning with the official FSM 811 controller active, while the
# joystick/SetVelocity path remains absent.  This is not a locomotion target.
arms-running-live:
	./scripts/r1-arms-running-live-test

# Local ROS service only: saves the current HMD/controller neutral while
# Deadman is released. It never constructs an SDK client or clears KILL.
arms-calibrate:
	./scripts/r1-arms-calibrate

# Save the current HMD neutral in an already-running dry/live head graph.  It
# only calls the local Trigger service and cannot create an SDK writer.
head-calibrate:
	./scripts/r1-head-calibrate

legs-live:
	./scripts/r1-locomotion-live-test

teleop-dry-run:
	./scripts/r1-teleop-dry-run

teleop-dry-run-smoke:
	./scripts/r1-teleop-dry-run-smoke

# Asserts the independent central software kill first, then requests bounded
# stop/release from the existing writer. It never starts a writer or enables motion.
robot-stop:
	./scripts/r1-robot-stop

robot-kill:
	./scripts/r1-robot-kill

robot-zero-torque:
	./scripts/r1-zero-torque

# Explicit full physical session. The script fails closed unless all required
# acknowledgements are already supplied; do not invoke it on a charging robot.
teleop-live:
	./scripts/r1-teleop-live

robot-pov: robot-pov-mock

robot-pov-mock:
	./scripts/robot-pov mock

robot-pov-exhibition:
	./scripts/robot-pov exhibition

robot-pov-bad-wifi:
	./scripts/robot-pov bad-wifi

robot-pov-r1:
	./scripts/robot-pov r1

robot-pov-r1-preflight:
	./scripts/robot-pov r1-preflight

robot-pov-preflight:
	./scripts/robot-pov preflight

# Short aliases for the video-only immersive POV workflow. These targets never
# start a robot-control node; they only select the camera/profile and viewer.
pov-demo:
	./scripts/robot-pov mock --profile balanced --port "$${R1_POV_DEMO_PORT:-18080}"

pov-vr:
	./scripts/robot-pov r1

pov-stereo-demo:
	./scripts/robot-pov mock --profile balanced --layout stereo --port "$${R1_POV_STEREO_DEMO_PORT:-18081}"

pov-high-quality:
	./scripts/robot-pov r1 --profile high

pov-low-latency:
	./scripts/robot-pov r1 --profile low-latency

pov-exhibition:
	./scripts/robot-pov exhibition

pov-preflight:
	./scripts/robot-pov r1-preflight

# Russian desktop operator panel. It only orchestrates existing commands; the
# default config is dry-run and physical live wrappers remain fail-closed.
operator-panel:
	./scripts/r1-operator-panel

# One-command exhibition entry points.  The foreground manager supervises
# video-only POV reconnects and logs every child.  Physical control still uses
# teleop-live -> robot-prepare and therefore retains all SDK/KILL/watchdog gates.
exhibition-panel:
	./scripts/r1-exhibition panel

exhibition-static:
	ROBOT_CONFIRM_STATIC_PREPARE=1 \
	R1_EXHIBITION_SESSION_MODE=deadman ./scripts/r1-exhibition static

exhibition-control:
	R1_EXHIBITION_SESSION_MODE="$${R1_EXHIBITION_SESSION_MODE:-session_arm}" \
	SAFETY_PROFILE=exhibition \
	ROBOT_EXHIBITION_SESSION=1 \
		./scripts/r1-exhibition control

# Compatibility name for the complete exhibition arms+legs session.  Physical
# HMD-to-head control remains a separately reviewed head-live stage.
exhibition-full-live:
	R1_EXHIBITION_SESSION_MODE="$${R1_EXHIBITION_SESSION_MODE:-session_arm}" \
	SAFETY_PROFILE=exhibition \
	ROBOT_EXHIBITION_SESSION=1 \
		./scripts/r1-exhibition control

exhibition-reconnect:
	./scripts/r1-exhibition reconnect

# Normal RUN -> LOCK transition.  The control graph stays prepared and the
# bridge changes to hold/zero in place, so no SDK or video restart is needed.
exhibition-lock:
	R1_EXHIBITION_SESSION_MODE=session_arm \
	SAFETY_PROFILE=exhibition \
	ROBOT_EXHIBITION_SESSION=1 \
		./scripts/r1-exhibition lock

# Reuse the already-checked control graph after a latched writer KILL.  The
# helper repeats calibration, current feedback/traffic gates and official
# prepare, but does not restart video, DDS readers or the live writer.
exhibition-rearm:
	R1_EXHIBITION_SESSION_MODE=session_arm \
	SAFETY_PROFILE=exhibition \
	ROBOT_EXHIBITION_SESSION=1 \
		./scripts/r1-exhibition rearm

exhibition-calibrate:
	R1_EXHIBITION_SESSION_MODE="$${R1_EXHIBITION_SESSION_MODE:-session_arm}" \
		./scripts/r1-exhibition-calibrate

exhibition-calibrate-arms:
	R1_EXHIBITION_SESSION_MODE="$${R1_EXHIBITION_SESSION_MODE:-session_arm}" \
		./scripts/r1-exhibition calibrate

exhibition-calibrate-head:
	./scripts/r1-head-calibrate

exhibition-stop:
	R1_EXHIBITION_SESSION_MODE="$${R1_EXHIBITION_SESSION_MODE:-session_arm}" \
		./scripts/r1-exhibition stop

exhibition-status:
	./scripts/r1-exhibition status

exhibition-mock-smoke:
	python3 -m pytest -q tests/test_exhibition_orchestrator.py

r1-panel-status:
	./scripts/r1-panel-status

r1-voice-preflight:
	./scripts/r1-voice-preflight

r1-sdk-warmup:
	./scripts/r1-sdk-warmup

# Optional user service: starts only the operator UI.  The companion offline
# session service remains video/bridge-only and never enables a live writer.
r1-operator-panel-autostart-install:
	install -Dm0644 systemd/user/r1-operator-panel.service \
		"$${XDG_CONFIG_HOME:-$${HOME}/.config}/systemd/user/r1-operator-panel.service"
	install -Dm0644 'desktop/Unitree R1 Панель оператора.desktop' \
		"$${HOME}/.local/share/applications/unitree-r1-operator-panel.desktop"
	install -Dm0755 'desktop/Unitree R1 Панель оператора.desktop' \
		"$${HOME}/Рабочий стол/Unitree R1 Панель оператора.desktop"
	systemctl --user daemon-reload
	systemctl --user enable --now r1-operator-panel.service

operator-panel-smoke:
	QT_QPA_PLATFORM=offscreen ./scripts/r1-operator-panel --smoke

operator-panel-check:
	QT_QPA_PLATFORM=offscreen python3 -m pytest -q tests/test_operator_panel.py tests/test_processes.py tests/test_operator_panel_service.py
	PYTHONPATH=ros2_ws/src/r1_robot_pov:$${PYTHONPATH:-} python3 -m pytest -q ros2_ws/src/r1_robot_pov/test

r1-camera-preflight:
	./scripts/r1-camera-preflight

robot-pov-deps:
	./scripts/install-robot-pov-deps

robot-pov-deps-offline:
	./scripts/install-robot-pov-deps --offline

robot-pov-cache-deps:
	./scripts/cache-robot-pov-deps

robot-pov-build:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install --packages-select r1_robot_pov'

robot-pov-check:
	bash -c 'source /opt/ros/humble/setup.bash && cd ros2_ws/src/r1_robot_pov && ament_flake8 r1_robot_pov test launch setup.py && ament_pep257 r1_robot_pov && python3 -m py_compile launch/robot_pov.launch.py setup.py && python3 -m pytest -q'

unity-telepresence-validate:
	./scripts/build-unity-telepresence validate

unity-telepresence-build:
	./scripts/build-unity-telepresence build
