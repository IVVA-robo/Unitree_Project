"""Static contracts for the fail-closed physical commissioning wrappers."""

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
    wrappers = sorted(SCRIPTS.glob('r1-*'))
    assert wrappers
    subprocess.run(
        ('bash', '-n', *(str(path) for path in wrappers)),
        check=True,
    )


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
    """An incomplete `topic list` result must not hide feedback or writers."""
    text = script('r1-sdk-preflight')
    assert 'ros2 topic type /r1/sdk/joint_states --no-daemon' in text
    assert 'sensor_msgs/msg/JointState --no-daemon' in text
    assert 'ros2 topic type "$forbidden" --no-daemon' in text
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
        "grep -Fxq '/r1_sdk_transport'",
        'ros2 param get /r1_sdk_transport sdk_enabled',
        'fresh_finite_reader_sample',
        'ros2 topic echo --once --full-length',
        'name_count} -eq 26',
        'position_count} -eq 26',
        'bad_value} -eq 0',
        'START_READER=false',
        'preflight_args+=(--no-reader)',
    ):
        assert required in live
    assert live.index("grep -Fxq '/r1_sdk_transport'") \
        < live.index('START_READER=false')
    assert live.index('fresh_finite_reader_sample') \
        < live.index('START_READER=false')


def test_preflight_never_uses_topic_type_as_reader_liveness():
    """Reader startup is based on node/service liveness, not graph residue."""
    text = script('r1-sdk-preflight')
    decision = text[text.index("section 'Read-only ROS transport'"):
                    text.index('# Query the exact topic directly.')]
    assert "grep -Fxq '/r1_sdk_transport'" in decision
    assert 'ros2 param list /r1_sdk_transport' in decision
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


def test_prepare_retries_read_only_parameter_discovery_fail_closed():
    """A transient empty graph sample may delay prepare, never bypass gates."""
    text = script('r1-robot-prepare')
    start = text.index('get_live_writer_param()')
    end = text.index('require_param transport', start)
    discovery = text[start:end]
    assert 'for attempt in 1 2 3 4' in discovery
    assert '--no-daemon --spin-time 1.0' in discovery
    assert "pattern='^String value is: .+$'" in discovery
    assert "pattern='^Boolean value is: (true|false)$'" in discovery
    assert 'grep -Eqi -- "${pattern}"' in discovery
    assert 'return 1' in discovery
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


def test_prepare_blocks_offcentre_or_moving_head_before_kill_release():
    """A known sideways seed must be rejected before StandUp is possible."""
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
    assert 'yaw_limit = 2.0071 if probe_mode else 0.35' \
        in text[sample_check:release]
    assert 'pitch_limit = 0.6283 if probe_mode else 0.25' \
        in text[sample_check:release]
    assert 'abs(sample[0]) > yaw_limit' in text[sample_check:release]
    assert 'abs(sample[1]) > pitch_limit' in text[sample_check:release]
    assert 'abs(sample[2]) > 0.05' in text[sample_check:release]
    assert 'abs(sample[3]) > 0.05' in text[sample_check:release]


def test_stop_and_kill_reach_writer_then_independent_supervisor():
    """Both stop layers remain explicit and the physical E-stop is documented."""
    stop = script('r1-robot-stop')
    kill = script('r1-robot-kill')
    assert '/r1/live_writer/stop' in stop
    assert '/r1/live_writer/kill' in kill
    for text in (stop, kill):
        assert '/r1/safety/emergency_stop' in text
        assert 'physical E-stop' in text


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
        'locomotion-live-test:',
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
