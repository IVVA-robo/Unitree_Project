"""Static contracts for the fail-closed physical commissioning wrappers."""

import os
from pathlib import Path
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]
PROJECT = PACKAGE.parents[2]
SCRIPTS = PROJECT / 'scripts'
MAKEFILE = (PROJECT / 'Makefile').read_text(encoding='utf-8')
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')


def script(name):
    """Return one staged wrapper as text."""
    return (SCRIPTS / name).read_text(encoding='utf-8')


def test_all_staged_wrappers_have_valid_bash_syntax():
    """Catch a broken safety wrapper before anyone reaches commissioning."""
    wrappers = [
        path for path in sorted(SCRIPTS.glob('r1-*'))
        if path.read_bytes().splitlines()[0] == b'#!/usr/bin/env bash'
    ]
    assert wrappers
    subprocess.run(
        ('bash', '-n', *(str(path) for path in wrappers)),
        check=True,
    )


def test_live_auto_setup_discovers_pico_and_restores_exact_offline_unit():
    """Interactive launch owns the reviewed bridge handoff for its lifetime."""
    text = script('r1-live-auto-run')
    for required in (
        'r1-exhibition-offline-handoff',
        'adb',
        'ROBOT_VR_SOURCE_IP',
        'ip -o -4 addr show dev wlan0',
        'UnityPlayerActivity',
        'trap restore_offline EXIT',
        '"${HANDOFF}" stop',
        '"${HANDOFF}" start',
    ):
        assert required in text
    assert 'pkill' not in text
    assert 'killall' not in text

    for wrapper in (
        'r1-arms-live-test',
        'r1-arms-running-live-test',
        'r1-head-live-test',
        'r1-head-running-live-test',
        'r1-locomotion-live-test',
        'r1-locomotion-127-probe',
    ):
        assert 'r1-live-auto-run' in script(wrapper)

    full = script('r1-teleop-live')
    assert 'r1-live-session" full' in full
    assert 'r1-live-auto-run' not in full


def test_physical_locomotion_uses_captured_run_and_excludes_phone_control():
    """The panel selects captured Run/FSM 811 without a competing phone stick."""
    text = script('r1-live-session')
    locomotion = text[text.index('locomotion)'):text.index('full)')]

    assert 'ENABLE_LOCOMOTION=true' in locomotion
    assert 'LOCOMOTION_COMMAND_MODE=wireless_controller' in locomotion
    assert 'require_exact ROBOT_CONFIRM_RUN_MODE 1' in text
    assert 'require_exact ROBOT_CONFIRM_NO_PHONE_CONTROL 1' in text
    assert 'SetFsmId calls used here: 4 and 811' in text
    assert 'acknowledgement authorizes the session to select Run/FSM 811 itself' in text
    assert 'locomotion_command_mode:="${LOCOMOTION_COMMAND_MODE}"' in text
    assert 'wireless_controller_rate_hz:="${WIRELESS_CONTROLLER_RATE_HZ}"' in text
    full = text[text.index('  full)'):text.index('  *) die', text.index('  full)'))]
    assert 'ENABLE_HEAD=true' in full
    assert 'ENABLE_ARMS=true' in full
    assert 'ENABLE_LOCOMOTION=true' in full


def test_status_127_probe_has_a_separate_force_stopped_app_gate():
    """The ordinary locomotion stage must never inherit this experiment."""
    text = script('r1-live-session')
    start = text.index('  locomotion-127-probe)')
    stage = text[start:text.index('  full)', start)]

    assert 'ENABLE_HEAD=false' in stage
    assert 'ENABLE_ARMS=false' in stage
    assert 'ENABLE_LOCOMOTION=true' in stage
    assert 'VELOCITY_STATUS_127_PROBE=true' in stage
    assert 'PREPARE_SPEED_MODE=1' in stage
    assert 'require_exact ROBOT_CONFIRM_VELOCITY_127_PROBE 1' in text
    assert 'ROBOT_CONFIRM_UNITREE_EXPLORE_CLOSED-} != 1' in text
    assert 'ROBOT_CONFIRM_NO_PHONE_CONTROL-} != 1' in text
    assert 'require_exact ROBOT_CONFIRM_AI_SPORT_1_0_2_154 1' in text
    assert 'LEG_SPEED_SCALE=0.75' in text
    assert (
        'velocity_status_127_probe_enabled:="${VELOCITY_STATUS_127_PROBE}"'
        in text
    )
    assert 'prepare_speed_mode:="${PREPARE_SPEED_MODE}"' in text


def test_arm_check_is_read_only_and_cannot_delegate_to_live_session():
    """The acknowledgement check must never launch the physical writer."""
    text = script('r1-robot-live-arm-check')
    assert 'r1-sdk-preflight' in text
    assert 'r1-live-session' not in text
    assert 'r1-teleop-live' not in text
    assert 'r1_live_writer_node' in text  # reject an already-running writer


def test_sdk_preflight_rejects_half_duplex_and_growing_rx_errors():
    """Carrier alone must not admit an unusable robot-facing Ethernet link."""
    text = script('r1-sdk-preflight')
    for required in (
        '/sys/class/net/$INTERFACE',
        'duplex',
        'rx_errors',
        'rx_length_errors',
        'rx_errors_delta == 0',
        'do not commission SDK writers',
    ):
        assert required in text


def test_sdk_preflight_half_duplex_override_is_explicit_and_cdc_only():
    """The operator override must not silently weaken arbitrary interfaces."""
    text = script('r1-sdk-preflight')
    assert 'R1_SDK_ALLOW_HALF_DUPLEX' in text
    assert 'ALLOW_HALF_DUPLEX == 1 && $driver_name == cdc_ether' in text
    assert 'rx_errors_delta == 0' in text
    assert 'telemetry gates remain mandatory' in text


def test_sdk_preflight_uses_direct_topic_discovery_for_safety_checks():
    """One coherent topic snapshot must expose feedback and forbidden writers."""
    text = script('r1-sdk-preflight')
    assert 'ros2 topic type /r1/sdk/joint_states --no-daemon' in text
    assert 'sensor_msgs/msg/JointState --no-daemon' in text
    assert 'ros2 topic list --no-daemon --spin-time 1.5' in text
    assert "coherent ROS topic snapshot was unavailable" in text
    assert 'command topic is present: $forbidden' in text
    assert 'topic list -t --no-daemon' not in text


def test_sdk_preflight_requires_fresh_zero_motorstate_health():
    """Finite joint coordinates alone cannot hide a reported motor fault."""
    text = script('r1-sdk-preflight')
    for required in (
        '/r1/sdk_transport/motors_healthy',
        'std_msgs/msg/Bool --no-daemon',
        'data: true',
        'motorstate_nonzero=0',
        'motorstate=none',
        'slot:code',
    ):
        assert required in text


def test_dry_run_wrappers_omit_optional_empty_launch_values():
    """An empty optional IP must use the launch default, not emit `name:=`."""
    dry_run = script('r1-teleop-dry-run')
    head = script('r1-head-live-dry-arm')
    assert 'if [[ -n ${VR_ALLOWED_SOURCE_IP} ]]' in dry_run
    assert 'launch_arguments+=("vr_allowed_source_ip:=${VR_ALLOWED_SOURCE_IP}")' \
        in dry_run
    assert 'if [[ -n ${VR_SOURCE_IP} ]]' in head
    assert 'launch_arguments+=("vr_source_ip:=${VR_SOURCE_IP}")' in head

    live = script('r1-live-session')
    assert 'VR_SOURCE_LAUNCH_ARG=()' in live
    assert 'if [[ -n ${VR_SOURCE_IP} ]]' in live
    assert 'VR_SOURCE_LAUNCH_ARG+=("vr_source_ip:=${VR_SOURCE_IP}")' \
        in live
    assert 'vr_source_ip:="${VR_SOURCE_IP}"' not in live


def test_ue200_driver_test_is_exact_nonpersistent_and_reversible():
    """The maintenance helper must not mutate an arbitrary USB/network device."""
    text = script('r1-ue200-driver-test')
    for required in (
        'VENDOR=2357',
        'PRODUCT=0602',
        'require_writer_idle',
        'require_rj45_unplugged',
        "printf '%s %s ff\\n'",
        'bConfigurationValue',
        'remove_id',
        'rollback_on_error',
        'configuration 2 uses cdc_ether',
    ):
        assert required in text
    assert '/etc/modprobe.d' not in text
    assert 'modprobe -r cdc_ether' not in text
    assert 'update-initramfs' not in text


def test_live_session_requires_every_gate_and_starts_killed():
    """Direct live entry cannot bypass acknowledgement or startup kill."""
    text = script('r1-live-session')
    for required in (
        'ROBOT_DRY_RUN',
        'ROBOT_ENABLE_ACTUATION',
        'ROBOT_CONFIRM_OFF_CHARGER',
        'ROBOT_CONFIRM_CLEAR_AREA',
        'ROBOT_CONFIRM_ESTOP_READY',
        'ROBOT_CONFIRM_COMMISSIONING',
        'ROBOT_COMMISSIONING_TOKEN',
        'ROBOT_VR_SOURCE_IP',
        'r1-sdk-preflight',
        '/r1/safety/emergency_stop',
    ):
        assert required in text
    assert 'transport:=sdk' in text
    assert 'send_commands:=true' in text
    assert 'profile:=slow-safe' in text
    assert 'head_absolute_envelope_confirmed:="${ENABLE_HEAD}"' in text
    assert 'R1_PHYSICAL_SDK_SESSION=1' in text


def test_ownership_probe_has_a_new_ack_and_legacy_recenter_is_blocked():
    """The micro-probe cannot inherit either generic or retired approval."""
    live = script('r1-live-session')
    prepare = script('r1-robot-prepare')
    assert 'head-probe)' in live
    head_probe = live[live.index('  head-probe)'):live.index('  recenter)')]
    assert 'PREPARE_ENTER_LOCOMOTION=true' in head_probe
    assert 'recenter)' in live
    assert 'full physical recenter is suspended' in live
    assert 'require_exact ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE 1' in live
    assert 'head_recenter_enabled:="${HEAD_OWNERSHIP_PROBE}"' in live
    assert 'head_recenter_confirmed:=false' in live
    assert 'head_ownership_probe_only:=true' in live
    assert 'head_ownership_probe_confirmed:="${HEAD_OWNERSHIP_PROBE}"' \
        in live
    assert 'require_exact ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE 1' in prepare
    assert 'head_ownership_probe_only' in prepare
    assert 'head_ownership_probe_confirmed' in prepare
    assert 'ROBOT_CONFIRM_HEAD_RECENTER' not in live
    assert 'ROBOT_CONFIRM_HEAD_RECENTER' not in prepare


def test_head_running_stage_enters_811_without_velocity_pipeline():
    live = script('r1-live-session')
    stage = live[live.index('  head-running)'):live.index('  arms)')]
    assert 'ENABLE_HEAD=true' in stage
    assert 'ENABLE_ARMS=false' in stage
    assert 'ENABLE_LOCOMOTION=false' in stage
    assert 'PREPARE_ENTER_LOCOMOTION=true' in stage
    wrapper = script('r1-head-running-live-test')
    assert 'r1-live-session" head-running' in wrapper
    assert 'SetVelocity' in wrapper
    assert 'head-running-live:' in MAKEFILE

    request = script('r1-robot-head-ownership-probe')
    for required in (
        'string value is: sdk',
        'ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE',
        'head_ownership_probe_only',
        'head_ownership_probe_confirmed',
        'head_recenter_confirmed',
        'head_probe_step_rad',
        'head_ownership_probe=pending_fresh_seed',
        'head_ownership_probe=zero_weight_discovery',
        'head_ownership_probe=weight_ramp_started',
        'head_ownership_probe=confirming_seed_return',
        'head_ownership_probe=passed terminal=true',
        'release_complete=true',
        r'head_release=.*weight=0',
        'baseline_ready = bool(node.statuses and node.armed and node.kill)',
        'probe-only writer unexpectedly armed before request',
        "central kill is not freshly clear before probe request",
        'writer never published fresh armed=true during probe',
        'abort_probe_if_needed',
        '/r1/live_writer/kill',
        '/r1/safety/emergency_stop',
        'armed=false after probe',
        'kill=true after probe',
    ):
        assert required in request
    trap = request.index('trap abort_probe_if_needed EXIT')
    ambiguous = request.index('PROBE_REQUESTED=true', trap)
    service = request.index("'/r1/live_writer/probe_head_ownership'", ambiguous)
    assert trap < ambiguous < service

    for legacy_name in (
        'r1-head-recenter-session',
        'r1-robot-head-recenter',
    ):
        legacy = script(legacy_name)
        assert 'Full physical head recenter is suspended.' in legacy
        assert 'No Unitree command was sent.' in legacy
        assert 'ros2 ' not in legacy
        assert 'r1-live-session' not in legacy


def test_full_stage_combines_reviewed_head_arms_and_wireless_locomotion():
    """The exhibition entry point uses one combined freshness barrier."""
    live = script('r1-live-session')
    stage = live[live.index('  full)'):live.index('  *) die', live.index('  full)'))]
    assert 'ENABLE_HEAD=true' in stage
    assert 'ENABLE_ARMS=true' in stage
    assert 'ENABLE_LOCOMOTION=true' in stage
    assert 'PREPARE_ENTER_LOCOMOTION=true' in stage
    assert 'LOCOMOTION_COMMAND_MODE=wireless_controller' in stage
    wrapper = script('r1-teleop-live')
    assert 'r1-live-session" full' in wrapper


def test_physical_graph_pins_fastdds_and_matched_unitree_cyclonedds():
    """ROS and SDK DDS implementations must never resolve one mixed ABI."""
    ros_env = script('r1-physical-ros-env')
    sdk_env = script('r1-unitree-sdk-env')
    assert 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' in ros_env
    assert 'librmw_fastrtps_cpp.so' in ros_env
    assert 'source "${_r1_env_dir}/r1-physical-ros-env"' in sdk_env
    assert 'libddsc.so.0 libddscxx.so.0' in sdk_env
    assert 'LD_LIBRARY_PATH="${R1_UNITREE_DDS_LIB_DIR}' in sdk_env

    for name in ('r1-live-session', 'r1-sdk-preflight', 'r1-sdk-readonly'):
        assert 'r1-unitree-sdk-env' in script(name), name
    for name in (
        'r1-robot-prepare',
        'r1-robot-stop',
        'r1-robot-kill',
        'r1-safety-supervisor',
    ):
        assert 'r1-physical-ros-env' in script(name), name


def test_physical_preflight_runs_isolated_abi_gate_before_robot_sdk():
    """A loader regression must block before a robot-facing reader starts."""
    text = script('r1-sdk-preflight')
    abi_gate = text.index('ros2 run r1_live_writer r1-sdk-abi-probe')
    physical_link = text.index("section 'Robot-facing Ethernet'")
    reader = text.index('setsid "$SCRIPT_DIR/r1-sdk-readonly"')
    assert abi_gate < physical_link < reader
    assert 'private loopback; it cannot route to the robot' in text
    assert 'refusing every physical SDK process' in text


def test_reader_reuse_requires_live_node_service_and_fresh_finite_feedback():
    """A stale DDS topic/type must never suppress reader startup."""
    live = script('r1-live-session')
    for required in (
        'r1-exhibition-reader-probe',
        'reader_probe_status',
        '10) START_READER=false',
        'START_READER=false',
        'preflight_args+=(--no-reader)',
    ):
        assert required in live
    assert live.index('r1-exhibition-reader-probe') \
        < live.index('START_READER=false')


def test_preflight_never_uses_topic_type_as_reader_liveness():
    """Reader startup is based on node/service liveness, not graph residue."""
    text = script('r1-sdk-preflight')
    decision = text[text.index("section 'Read-only ROS transport'"):
                    text.index('# Query the exact topic directly.')]
    assert 'r1-exhibition-reader-probe' in decision
    assert 'reader_probe_status == 10' in decision
    assert 'reader_probe_status != 0' in decision
    assert 'topic type /r1/sdk/joint_states' not in decision
    assert 'inconsistent/stale /r1_sdk_transport discovery' in decision


def test_temporary_reader_owns_and_cleans_its_complete_process_group():
    """A ros2-launch child must not survive preflight cleanup and look live."""
    text = script('r1-sdk-preflight')
    assert 'setsid "$SCRIPT_DIR/r1-sdk-readonly"' in text
    for signal in ('INT', 'TERM', 'KILL'):
        assert f'kill -{signal} -- "-$READER_PID"' in text
    assert 'wait "$READER_PID"' in text
    assert '--qos-durability volatile' in text


def test_prepare_only_calls_an_existing_reviewed_writer():
    """Prepare must not construct a launch or call the legacy dry-run plan."""
    text = script('r1-robot-prepare')
    assert 'ros2 launch' not in text
    assert 'r1_prepare' not in text
    assert '/r1/live_writer/reset_kill' in text
    assert '/r1/live_writer/prepare' in text
    assert '/r1/safety/set_kill' in text
    assert 'ros2 service list --no-daemon' in text
    assert '--show-types' in text
    assert 'required_service_graph_ready' in text
    discovery = text[text.index("service_graph=''"):
                     text.index('for service in', text.index("service_graph=''"))]
    assert '|| true' not in discovery
    assert "service_graph=''" in discovery
    assert 'ros2 service type' not in '\n'.join(
        line for line in text.splitlines() if not line.lstrip().startswith('#')
    )
    assert 'transport' in text and 'string value is: sdk' in text
    assert 'send_commands' in text and 'boolean value is: true' in text
    assert 'calibrated=(true|True)' in text


def test_live_session_reuses_only_a_live_authorized_killed_supervisor():
    """A stale dry-run supervisor must not inherit a physical live session."""
    text = script('r1-live-session')
    reuse = text[text.index('if [[ ${START_SUPERVISOR} == false ]]'):
                 text.index("echo 'R1 PHYSICAL WRITER SESSION")]
    emergency_stop = reuse.index('/r1/safety/emergency_stop')
    status = reuse.index('/r1/safety/get_status')
    assert emergency_stop < status
    assert (
        'mode=live-authorized dry_run=false enable_actuation=true '
        'off_charger=true clear_area=true estop_ready=true'
        in reuse
    )
    assert 'kill=true reason=emergency_stop_service' in reuse
    assert 'non-live or mismatched policy' in reuse


def test_prepare_retries_read_only_parameter_discovery_fail_closed():
    """A transient empty graph sample may delay prepare, never bypass gates."""
    text = script('r1-robot-prepare')
    start = text.index("parameter_dump=''")
    end = text.index('require_param transport', start)
    discovery = text[start:end]
    assert 'for attempt in 1 2 3 4' in discovery
    assert 'ros2 param dump /r1_live_writer' in discovery
    assert '--no-daemon --spin-time 1.5' in discovery
    assert "[[ -n ${parameter_dump} ]]" in discovery
    assert 'dump_scalar()' in discovery
    assert '[[ ${value} == true || ${value} == false ]]' in discovery
    assert 'return 1' in discovery
    assert text.count('ros2 param dump /r1_live_writer') == 1
    assert 'get_live_writer_param enable_head boolean' in text
    assert 'get_live_writer_param enable_locomotion boolean' in text
    assert 'get_live_writer_param enable_arms boolean' in text
    assert 'get_live_writer_param arm_topic string' in text
    assert "die 'live writer parameter enable_head is unavailable or not boolean'" in text
    assert "die 'live writer parameter enable_locomotion is unavailable or not boolean'" in text
    assert "die 'live writer parameter enable_arms is unavailable or not boolean'" in text
    assert "die 'live writer parameter arm_topic is unavailable or not string'" in text
    assert 'require_param head_absolute_envelope_confirmed boolean' in text
    assert 'get_live_writer_param "${name}" "${expected_kind}" || true' not in text


def test_prepare_waits_for_the_structured_post_prepare_state():
    """Do not couple the wrapper to changeable prose returned by the SDK."""
    text = script('r1-robot-prepare')
    assert (
        'prepare_result=.*[[:space:]]prepare_rearm=await_deadman_release'
        in text
    )
    assert 'prepare_result=.*stable FSM 4' not in text


def test_prepare_reports_stand_only_without_claiming_start_for_arms():
    """The operator message must distinguish arms-only FSM 4 from FSM 811."""
    text = script('r1-robot-prepare')
    assert "if grep -Fqi 'boolean value is: true'" in text
    assert "<<<\"${prepare_enter_locomotion}\"" in text
    assert 'get_live_writer_param prepare_enter_locomotion boolean' in text
    assert 'enable_locomotion' in text
    assert 'official R1 StandUp is confirmed by stable FSM 4.' in text
    assert (
        'Locomotion is disabled: Start/FSM 811 and velocity commands were not requested.'
        in text
    )
    assert 'head/arms-only mode skips the neutral-velocity gate.' in text


def test_arms_running_wrapper_is_explicit_and_fail_closed():
    """Running-controller arm commissioning never enables stick velocity."""
    text = script('r1-arms-running-live-test')
    assert 'r1-live-session' in text
    assert 'arms-running' in text
    live = script('r1-live-session')
    stage = live[live.index('  arms-running)'):live.index('  head-probe)')]
    assert 'ENABLE_ARMS=true' in stage
    assert 'ENABLE_LOCOMOTION=false' in stage
    assert 'PREPARE_ENTER_LOCOMOTION=true' in stage
    assert 'SetVelocity' not in stage
    assert 'arms-running-live:' in MAKEFILE
    assert './scripts/r1-arms-running-live-test' in MAKEFILE


def test_prepare_auto_centers_stable_exhibition_head_and_blocks_invalid_or_moving_seed():
    """Exhibition accepts bounded centering; wider or moving seeds still block."""
    text = script('r1-robot-prepare')
    sample_check = text.index("class HeadSeedCheck(Node):")
    five_samples = text.index('len(node.samples) < 5', sample_check)
    seed_window = text.index(
        'physical head is outside the accepted seed envelope',
        five_samples,
    )
    velocity_limit = text.index(
        'physical head is not stationary enough to seed', seed_window
    )
    release = text.index(
        "echo 'Releasing the software kill", velocity_limit
    )
    assert sample_check < five_samples < seed_window < velocity_limit < release
    assert "'head_yaw_joint', 'head_pitch_joint'" in text[sample_check:release]
    assert "len(message.name) != len(message.velocity)" in text[sample_check:release]
    precheck = text[sample_check:release]
    assert 'auto_center_mode = sys.argv[2].lower()' in precheck
    assert 'feedback_margin = 0.01' in precheck
    assert 'yaw_limit = 2.0071 + feedback_margin' in precheck
    assert 'pitch_limit = 0.6283 + feedback_margin' in precheck
    assert 'yaw_limit = 0.35' in precheck
    assert 'pitch_limit = 0.25' in precheck
    assert 'abs(sample[0]) > yaw_limit' in text[sample_check:release]
    assert 'abs(sample[1]) > pitch_limit' in text[sample_check:release]
    assert 'abs(sample[2]) > 0.05' in text[sample_check:release]
    assert 'abs(sample[3]) > 0.05' in text[sample_check:release]
    assert '[AUTO_CENTER]' in precheck
    assert 'head_auto_center_required=true' in text[sample_check:release]

    prepare = text.index('call_trigger /r1/live_writer/prepare')
    wait_prepare = text.index('wait_for_prepare_confirmation', prepare)
    wait_center = text.index('wait_for_head_auto_center', wait_prepare)
    assert prepare < wait_prepare < wait_center
    assert 'head_auto_center=complete' in text
    assert 'head_auto_center_complete=true' in text
    # StandUp/Start may center the head after the wider precheck. A fresh
    # stable normal seed is an equally valid outcome, not a 45s timeout.
    assert 'head_tracking_seed_verified=true' in text


def test_stop_and_kill_assert_central_latch_before_writer_cleanup():
    """The independent latch must not wait behind an unresponsive writer."""
    stop = script('r1-robot-stop')
    kill = script('r1-robot-kill')
    assert '/r1/live_writer/stop' in stop
    assert '/r1/live_writer/kill' in kill
    for text, writer_endpoint in (
        (stop, '/r1/live_writer/stop'),
        (kill, '/r1/live_writer/kill'),
    ):
        assert '/r1/safety/emergency_stop' in text
        assert 'physical E-stop' in text
        assert text.index('/r1/safety/emergency_stop') \
            < text.index(writer_endpoint)
        central_call = text[text.index('kill_response='):text.index(writer_endpoint)]
        assert '"${TRIGGER_CLIENT}" /r1/safety/emergency_stop 4' \
            in central_call
        assert 'timeout ' not in text
        assert 'ros2 service call' not in text


def test_bounded_trigger_client_owns_one_deadline_without_rclpy_signal_shutdown():
    """STOP diagnostics must not be corrupted by GNU timeout SIGTERM."""
    text = script('r1-call-trigger-service')
    for required in (
        'time.monotonic()',
        'SignalHandlerOptions.NO',
        'client.wait_for_service(',
        'client.call_async(Trigger.Request())',
        'executor.spin_once(',
        '[UNAVAILABLE]',
        '[TIMEOUT]',
    ):
        assert required in text
    assert 'subprocess' not in text
    assert 'os.kill' not in text


def test_prepare_failure_relocks_central_supervisor_before_writer_cleanup():
    """A prepare error must latch central kill without waiting on the writer."""
    text = script('r1-robot-prepare')
    relock = text[text.index('emergency_relock() {'):
                  text.index('trap emergency_relock ERR')]
    central = relock.index('/r1/safety/emergency_stop')
    writer = relock.index('/r1/live_writer/kill')
    assert central < writer
    assert relock.index('timeout 4s ros2 service call') < central
    assert relock.index('timeout 5s ros2 service call', central) < writer


def test_stop_and_kill_reach_central_latch_when_writer_is_unavailable(tmp_path):
    """A missing writer must not make the independent kill path unreachable."""
    fake_client = """#!/usr/bin/env bash
set -eu
endpoint=$1
duration=$2
printf '%s %s\\n' "${duration}" "${endpoint}" >> "${R1_TEST_CALL_LOG}"
if [[ ${endpoint} == /r1/safety/emergency_stop ]]; then
  printf 'response:\\n  success: true\\n  message: central latch asserted\\n'
  exit 0
fi
printf '[UNAVAILABLE] %s was not available within %ss\\n' \
  "${endpoint}" "${duration}" >&2
exit 2
"""

    for name, writer_endpoint in (
        ('r1-robot-stop', '/r1/live_writer/stop'),
        ('r1-robot-kill', '/r1/live_writer/kill'),
    ):
        project = tmp_path / name
        scripts = project / 'scripts'
        install = project / 'ros2_ws' / 'install'
        scripts.mkdir(parents=True)
        install.mkdir(parents=True)
        staged_script = scripts / name
        staged_script.write_text(script(name), encoding='utf-8')
        trigger_client = scripts / 'r1-call-trigger-service'
        trigger_client.write_text(fake_client, encoding='utf-8')
        trigger_client.chmod(0o755)
        (scripts / 'r1-physical-ros-env').write_text(':\n', encoding='utf-8')
        (install / 'setup.bash').write_text(':\n', encoding='utf-8')

        call_log = project / 'calls.log'
        environment = os.environ.copy()
        environment['R1_TEST_CALL_LOG'] = str(call_log)
        result = subprocess.run(
            ('bash', str(staged_script)),
            check=False,
            capture_output=True,
            env=environment,
            text=True,
            timeout=5,
        )

        assert result.returncode == 1
        assert call_log.read_text(encoding='utf-8').splitlines() == [
            '4 /r1/safety/emergency_stop',
            f'8 {writer_endpoint}',
        ]
        assert 'central latch asserted' in result.stdout
        assert '[UNAVAILABLE]' in result.stdout
        assert "rcl node's context is invalid" not in result.stdout
        assert 'could not be confirmed on every layer' in result.stderr


def test_stop_and_kill_reject_unsuccessful_trigger_responses(tmp_path):
    """A completed Trigger with success=false is never accepted as STOP."""
    project = tmp_path / 'rejected'
    scripts = project / 'scripts'
    install = project / 'ros2_ws' / 'install'
    scripts.mkdir(parents=True)
    install.mkdir(parents=True)
    staged_script = scripts / 'r1-robot-stop'
    staged_script.write_text(script('r1-robot-stop'), encoding='utf-8')
    trigger_client = scripts / 'r1-call-trigger-service'
    trigger_client.write_text(
        """#!/usr/bin/env bash
set -eu
printf 'response:\\n  success: false\\n  message: cleanup pending\\n'
exit 1
""",
        encoding='utf-8',
    )
    trigger_client.chmod(0o755)
    (scripts / 'r1-physical-ros-env').write_text(':\n', encoding='utf-8')
    (install / 'setup.bash').write_text(':\n', encoding='utf-8')
    result = subprocess.run(
        ('bash', str(staged_script)),
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
    assert result.stdout.count('success: false') == 2
    assert 'could not be confirmed on every layer' in result.stderr


def test_arm_calibration_is_local_and_requires_deadman_release():
    """Calibration may save VR neutral but must never authorize actuation."""
    text = script('r1-arms-calibrate')
    deadman = text.index('/vr/teleop/active')
    calibrate = text.index(
        'ros2 service call /vr/calibrate_body', deadman
    )
    assert deadman < calibrate
    assert "data:[[:space:]]*false" in text[deadman:calibrate]
    assert 'r1-physical-ros-env' in text
    assert 'r1-unitree-sdk-env' not in text
    assert 'ROBOT_ENABLE_ACTUATION' not in text
    assert '/r1/live_writer/reset_kill' not in text
    assert '/r1/safety/set_kill' not in text


def test_exhibition_calibration_captures_arms_then_head_without_sdk():
    """Complete RUN startup must save both local VR neutral references."""
    wrapper = script('r1-exhibition-calibrate')
    text = script('r1-exhibition-ros-client')
    arms = text.index('"/vr/calibrate_body"')
    head = text.index('"/r1/head/calibrate_neutral"', arms)
    assert arms < head
    assert 'r1-exhibition-ros-client' in wrapper
    assert 'r1-unitree-sdk-env' not in wrapper + text
    assert 'ROBOT_ENABLE_ACTUATION' not in text
    assert '/r1/live_writer/reset_kill' not in text


def test_one_live_domain_and_required_make_targets_are_preserved():
    """Operator commands must discover one another in the same ROS graph."""
    for name in (
        'r1-live-session',
        'r1-robot-live-arm-check',
        'r1-robot-prepare',
        'r1-robot-stop',
        'r1-robot-kill',
        'r1-safety-supervisor',
    ):
        text = script(name)
        assert 'R1_LIVE_ROS_DOMAIN_ID' in text
        assert '89' not in text
    for target in (
        'robot-readonly-preflight:',
        'robot-live-arm-check:',
        'robot-prepare:',
        'head-live-dry-arm:',
        'head-live-test:',
        'arms-calibrate:',
        'arms-running-live:',
        'locomotion-live-test:',
        'locomotion-127-probe:',
        'teleop-live:',
        'robot-stop:',
        'robot-kill:',
    ):
        assert target in MAKEFILE


def test_mock_smoke_is_loopback_only_and_cannot_reuse_live_channels():
    """Mock integration coverage must be unreachable from robot networks."""
    text = script('r1-live-writer-mock-smoke')
    assert 'unshare --user --map-root-user --net' in text
    assert 'assert_loopback_only_namespace' in text
    assert 'ROS_LOCALHOST_ONLY=1' in text
    assert 'udp_bind_address:=127.0.0.1' in text
    assert "('127.0.0.1', port)" in text
    assert '-p transport:=mock' in text
    assert '-p network_interface:=mock-only' in text
    assert 'transport:=sdk' not in text
    assert '10#${value} >= 230 && 10#${value} <= 232' in text
    assert '10#${value} != 9090' in text


def test_mock_smoke_cleans_a_group_even_after_its_leader_exits():
    """Cleanup must target the PGID, not depend on a live leader PID."""
    text = script('r1-live-writer-mock-smoke')
    assert 'process_group_alive()' in text
    assert 'kill -0 -- "-$1"' in text
    assert 'while process_group_alive "${pid}"' in text
    assert 'process group ${pid} survived KILL' in text
    assert 'stop_process_group "${UPSTREAM_PID}" || cleanup_failed=true' in text


def test_mock_smoke_exercises_rearm_and_independent_stop_causes():
    """The smoke has negative controls, not only a successful prepare path."""
    text = script('r1-live-writer-mock-smoke')
    for required in (
        'prepare_rearm=await_deadman_release',
        'prepare_rearm=await_neutral_velocity',
        'publish_non_neutral_velocity_once',
        'prepare_rearm=await_deadman_press',
        'prepare_rearm=ready',
        'stale_commands_discarded_armed',
        'kill_still_latched_armed',
        'deadman_released_armed',
        'start_command_feeder head',
        'start_command_feeder velocity',
        'stop_head_command_feeder',
        'stop_velocity_command_feeder',
        'watchdog_head_command_missing_stale_or_invalid',
        'watchdog_velocity_missing_stale_or_invalid',
    ):
        assert required in text


def test_probe_mock_smoke_is_loopback_only_and_covers_claim_return_cleanup():
    """The ownership micro-probe needs a permanent process-level mock test."""
    text = script('r1-head-recenter-mock-smoke')
    for required in (
        'unshare --user --map-root-user --net',
        'current_network_namespace',
        'caller_network_namespace',
        'probe smoke is not in a private network namespace',
        'private namespace exposes non-loopback interfaces',
        'ROS_LOCALHOST_ONLY=1',
        "'transport:=mock'",
        "'head_recenter_enabled:=true'",
        "'head_recenter_confirmed:=false'",
        "'head_ownership_probe_only:=true'",
        "'head_ownership_probe_confirmed:=true'",
        'ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE',
        "writer_environment.pop('R1_PHYSICAL_SDK_SESSION', None)",
        'full physical recenter is suspended',
        'head_ownership_probe=zero_weight_discovery',
        'head_ownership_probe=weight_ramp_started',
        'head_ownership_probe=full_weight_seed_hold',
        'head_ownership_probe=outbound_feedback_confirmed',
        'head_ownership_probe=confirming_seed_return',
        'head_ownership_probe=passed terminal=true',
        'MAX_PROBE_STEP = 0.005',
        'probe did not command a return to the exact seed',
        'head_release=mock ArmSdk weight release recorded',
        'probe completion did not request the central kill',
        'central safety supervisor did not latch kill=true',
        'writer remained armed after probe completion',
        'head_ownership_probe_failed cause=feedback_velocity_exceeded',
        'velocities[12] = 0.041',
    ):
        assert required in text
    assert "'transport:=sdk'" not in text


def test_offline_commissioning_check_cannot_enter_a_physical_path():
    """The aggregate regression command must remain software-only."""
    text = script('r1-offline-commissioning-check')
    commands = '\n'.join(
        line for line in text.splitlines()
        if not line.lstrip().startswith('#')
    )
    for required in (
        'ROBOT_DRY_RUN=1',
        'ROBOT_ENABLE_ACTUATION=0',
        'ROBOT_CONFIRM_OFF_CHARGER=0',
        'unset R1_PHYSICAL_SDK_SESSION',
        'unshare --user --map-root-user --net',
        'current_network_namespace',
        'caller_network_namespace',
        'private namespace exposes non-loopback interface',
        'r1-teleoperation-build',
        'r1-teleoperation-check',
        'r1-sdk-abi-probe',
        'r1-live-writer-mock-smoke',
        'r1-head-ownership-probe-mock-smoke',
        'teleop-dry-run-smoke',
        'r1-sim-smoke',
        'robot_processes',
        'r1_arm_sdk_dds_example',
        'r1_loco_client',
        'rt/(arm_sdk|lowcmd)',
        'sport([[:space:]]|$)',
    ):
        assert required in commands
    for forbidden in (
        'r1-sdk-preflight',
        'robot-readonly-preflight',
        'r1-live-session',
        'head-live-test',
        'locomotion-live-test',
        'teleop-live',
        'robot-prepare',
    ):
        assert forbidden not in commands


def test_offline_commissioning_make_recipes_remain_nonphysical():
    """Allowed aggregate targets may not be redirected to a live wrapper."""
    expected_recipes = {
        'r1-teleoperation-build': 'colcon build --symlink-install',
        'r1-teleoperation-check': 'colcon test --packages-up-to',
        'r1-sdk-abi-probe': 'ros2 run r1_live_writer r1-sdk-abi-probe',
        'r1-live-writer-mock-smoke': './scripts/r1-live-writer-mock-smoke',
        'r1-head-ownership-probe-mock-smoke': (
            './scripts/r1-head-ownership-probe-mock-smoke'
        ),
        'teleop-dry-run-smoke': './scripts/r1-teleop-dry-run-smoke',
        'r1-sim-smoke': './scripts/r1-sim-smoke',
    }
    make_lines = MAKEFILE.splitlines()
    for target, expected in expected_recipes.items():
        header = f'{target}:'
        index = make_lines.index(header)
        recipe = []
        for line in make_lines[index + 1:]:
            if line and not line.startswith(('\t', ' ')):
                break
            if line.startswith('\t'):
                recipe.append(line.strip())
        joined = '\n'.join(recipe)
        assert expected in joined, target
        for forbidden in (
            'r1-sdk-preflight',
            'r1-live-session',
            'r1-teleop-live',
            'robot-prepare',
            'head-live-test',
            'locomotion-live-test',
        ):
            assert forbidden not in joined, (target, forbidden)


def test_sdk_builds_override_stale_cmake_cache_and_audit_install():
    """Incremental builds must converge both SDK consumers on one root."""
    for target in (
        'r1-teleoperation-build:',
        'r1-sdk-build:',
        'r1-live-writer-build:',
    ):
        start = MAKEFILE.index(target)
        next_target = MAKEFILE.find('\n\n', start)
        recipe = MAKEFILE[start:next_target]
        assert '-DUNITREE_SDK_ROOT="$(R1_UNITREE_SDK_ROOT)"' in recipe

    offline = script('r1-offline-commissioning-check')
    for required in (
        'audit_installed_sdk_paths',
        'UNITREE_SDK_ROOT:PATH=',
        'readelf -d',
        "VENDOR_DDS_DIRECTORY = '",
        '--abi-check-only',
        'both installed SDK consumers use one selected vendor DDS directory',
    ):
        assert required in offline


def test_staged_script_contract_is_registered_with_ament():
    """The wrapper contract must run under colcon test, not just direct pytest."""
    assert (
        'ament_add_pytest_test(test_staged_scripts test/test_staged_scripts.py)'
        in CMAKE
    )
