import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / 'src' / 'r1_live_writer_node.cpp').read_text()
TRANSPORT = (ROOT / 'src' / 'transport.cpp').read_text()
TRANSPORT_HEADER = (ROOT / 'include' / 'r1_live_writer' / 'transport.hpp').read_text()
CONFIG = (ROOT / 'config' / 'r1_live_writer.yaml').read_text()
LAUNCH = (ROOT / 'launch' / 'r1_live_writer.launch.py').read_text()


def function_body(source: str, signature: str) -> str:
    """Return one C++ function body for focused, order-sensitive contracts."""
    signature_index = source.index(signature)
    opening_brace = source.index('{', signature_index)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError(f'unterminated C++ function: {signature}')


def test_defaults_are_mock_and_all_physical_features_are_off():
    assert 'transport: "mock"' in CONFIG
    assert 'send_commands: false' in CONFIG
    assert 'enable_head: false' in CONFIG
    assert 'enable_arms: false' in CONFIG
    assert 'enable_locomotion: false' in CONFIG
    assert 'enable_prepare: false' in CONFIG
    assert 'prepare_enter_locomotion: false' in CONFIG
    assert "default_value='mock'" in LAUNCH
    assert "default_value='false'" in LAUNCH


def test_physical_armsdk_is_locked_to_vendor_required_100_hz():
    """ArmSdk sends and its bounded release must keep the 10 ms cadence."""
    assert 'publish_rate_hz: 100.0' in CONFIG
    assert 'declare_parameter<double>("publish_rate_hz", 100.0)' in NODE

    validation = function_body(NODE, 'void validate_parameters() const')
    assert 'physical R1 ArmSdk control requires publish_rate_hz=100 Hz' in validation
    assert '(enable_head_ || enable_arms_)' in validation

    release = function_body(
        TRANSPORT, 'TransportResult SdkTransport::release_head('
    )
    assert 'constexpr auto kReleasePeriod = std::chrono::milliseconds(10)' in release
    assert 'constexpr int kReleaseSteps = 100' in release
    assert 'constexpr int kZeroBurstFrames = 30' in release


def test_arm_feedback_follow_guard_latches_existing_fail_closed_path():
    """A queued arm frame is insufficient without same-direction feedback."""
    arms = function_body(NODE, 'void process_arms(')
    timer = function_body(NODE, 'void on_timer()')
    guard = function_body(NODE, 'std::string arm_feedback_follow_failure(')
    record = function_body(NODE, 'void record_arm_feedback_follow_target(')

    assert 'reset_arm_feedback_follow(held_head_target_)' in arms
    assert 'record_arm_feedback_follow_target(held_head_target_' in arms
    assert 'arm_feedback_follow_failure(now)' in timer
    assert 'safe_stop_outputs(arm_follow_reason, true)' in timer
    assert 'arm_feedback_follow_timeout' in guard
    assert 'state_fresh(now)' in guard
    assert 'arm_response_guard_.observe(' in guard
    assert 'feedback, state_sequence_' in guard
    assert 'ArmResponseGuard::kRequiredProgressSamples' in guard
    assert 'arm_response_guard_.record_target(' in record
    assert 'command, feedback, state_sequence_' in record
    assert 'failure->target' in guard
    assert 'failure->feedback' in guard
    assert 'failure->command_reference' in guard
    assert 'failure->observation_target' in guard
    assert 'arm_feedback_follow_min_progress_rad_' in guard
    assert 'arm_feedback_follow_timeout_sec_' in guard


def test_arm_feedback_authority_guard_accepts_small_repeatable_motion():
    """The guard proves authority with fresh samples, not servo accuracy."""
    assert 'arm_feedback_follow_min_progress_rad: 0.0015' in CONFIG
    validation = function_body(NODE, 'void validate_parameters() const')
    assert 'arm_feedback_follow_min_progress_rad_ < 0.001' in validation

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    assert 'clear_arm_feedback_follow()' in safe_stop


def test_arm_rearm_captures_first_vr_target_before_allowing_delta():
    """Re-arm must not replay an already-offset controller pose."""
    arms = function_body(NODE, 'void process_arms(')
    freshness = function_body(NODE, 'void clear_command_freshness()')
    assert 'ArmNeutralHold' in NODE
    assert 'arm_neutral_hold_.arm(' in arms
    assert 'capture_first_target(latest_arm_)' in arms
    assert 'arm_neutral_hold=captured_first_vr_target' in arms
    assert 'arm_neutral_hold_.target_for(latest_arm_)' in arms
    assert 'publish_debug_arm(last_arm_command_)' in arms
    assert 'arm_neutral_hold_.reset()' in freshness


def test_arm_rearm_keeps_locomotion_disabled():
    assert 'enable_locomotion: false' in CONFIG
    assert 'enable_locomotion_' in NODE
    assert 'process_velocity' in NODE


def test_locomotion_command_refresh_is_bounded_without_delaying_zero_stop():
    """The 100 Hz arm loop must use each locomotion transport's cadence."""
    assert 'locomotion_rpc_rate_hz: 10.0' in CONFIG
    assert 'wireless_controller_rate_hz: 20.0' in CONFIG
    assert 'declare_parameter<double>(\n      "locomotion_rpc_rate_hz", 10.0)' in NODE

    validation = function_body(NODE, 'void validate_parameters() const')
    assert 'locomotion_rpc_rate_hz_ < 2.0' in validation
    assert 'locomotion_rpc_rate_hz_ > 10.0' in validation
    assert 'wireless_controller_rate_hz_ < 15.0' in validation
    assert 'wireless_controller_rate_hz_ > 25.0' in validation

    velocity = function_body(NODE, 'void process_velocity(')
    zero_branch = velocity.index('if (is_zero(shaped))')
    stop_call = velocity.index('stop_locomotion()', zero_branch)
    command_period = velocity.index(
        'const double command_period_sec = 1.0 / locomotion_command_rate_hz',
        stop_call,
    )
    throttled_return = velocity.index(
        'age(send_now, last_velocity_rpc_send_) < command_period_sec',
        command_period,
    )
    velocity_send = velocity.index('transport_->set_velocity(', throttled_return)
    timestamp = velocity.index('last_velocity_rpc_send_ =', velocity_send)
    assert zero_branch < stop_call < command_period < throttled_return
    assert throttled_return < velocity_send < timestamp

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    assert 'last_velocity_rpc_send_ = SteadyClock::time_point{}' in safe_stop


def test_writer_consumes_only_reviewed_safe_pipeline_topics():
    required = (
        '/r1_hardware_adapter/debug/head/joint_trajectory',
        '/r1_kinematics_control/debug/arm_trajectory',
        '/r1/locomotion_dry_run/debug/cmd_vel',
        '/r1/sdk/joint_states',
        '/r1/sdk_transport/motors_healthy',
        '/vr/teleop/active',
        '/r1/safety/kill',
        '/r1/safety/kill_request',
    )
    for topic in required:
        assert topic in CONFIG


def test_physical_writer_requires_fresh_motor_health_everywhere():
    """Mock stays independent; physical authorization and watchdogs fail closed."""
    for token in (
        'motor_health_required()',
        'motor_health_fresh(now)',
        'on_motor_health',
        'motor_health_reported_unhealthy',
        'watchdog_motor_health_missing_stale_or_unhealthy',
        'result.motor_health_required = motor_health_required()',
        'result.motors_healthy_fresh = motor_health_fresh(now)',
        'deadman_fresh(now) && state_fresh(now) && motor_health_fresh(now)',
    ):
        assert token in NODE
    assert 'transport_name_ == "sdk" && send_commands_' in NODE
    assert 'motor_health_timeout_sec: 0.50' in CONFIG
    assert "default_value='/r1/sdk_transport/motors_healthy'" in LAUNCH
    assert "'motor_health_topic': LaunchConfiguration(" in LAUNCH

    callback = function_body(NODE, 'void on_motor_health(')
    assert 'motor_health_required()' in callback
    assert 'outputs_active() || prepare_in_progress_' in callback
    assert 'safe_stop_outputs("motor_health_reported_unhealthy", true)' \
        in callback

    authorization = function_body(NODE, 'AuthorizationInput authorization_input(')
    assert 'result.motor_health_required = motor_health_required()' \
        in authorization
    assert 'result.motors_healthy_fresh = motor_health_fresh(now)' \
        in authorization

    for watchdog_signature in (
        'std::string active_watchdog_failure(',
        'std::string prepare_watchdog_failure(',
    ):
        watchdog = function_body(NODE, watchdog_signature)
        assert 'motor_health_fresh(now)' in watchdog
        assert 'watchdog_motor_health_missing_stale_or_unhealthy' in watchdog


def test_sdk_uses_official_r1_interfaces_and_no_leg_joint_writer():
    assert 'unitree::robot::r1::LocoClient' in TRANSPORT
    assert 'SetVelocity' in TRANSPORT
    assert 'StopMove' in TRANSPORT
    assert 'StandUp' in TRANSPORT
    assert 'Start' in TRANSPORT
    assert 'unitree::robot::r1::publisher::ArmSdk' in TRANSPORT
    forbidden = 'rt/' + 'lowcmd'
    assert forbidden not in TRANSPORT
    assert forbidden not in NODE


def test_arm_sdk_is_seeded_before_command_and_all_13_fields_are_named():
    assert 'command rejected before 13-field seed' in TRANSPORT
    assert 'seeded all 13 fields from fresh JointState' in TRANSPORT
    for name in (
        'left_shoulder_pitch_joint', 'right_wrist_roll_joint',
        'waist_yaw_joint', 'head_pitch_joint', 'head_yaw_joint',
    ):
        assert name in NODE


def test_weighted_arm_api_is_explicit_and_legacy_calls_remain_full_weight():
    for class_name in ('RobotTransport', 'MockTransport', 'SdkTransport'):
        class_start = TRANSPORT_HEADER.index(f'class {class_name}')
        class_end = TRANSPORT_HEADER.index('};', class_start)
        declaration = TRANSPORT_HEADER[class_start:class_end]
        assert 'seed_head_weighted(' in declaration
        assert 'command_head_weighted(' in declaration

    mock_seed = function_body(
        TRANSPORT, 'TransportResult MockTransport::seed_head('
    )
    mock_command = function_body(
        TRANSPORT, 'TransportResult MockTransport::command_head('
    )
    sdk_seed = function_body(
        TRANSPORT, 'TransportResult SdkTransport::seed_head('
    )
    sdk_command = function_body(
        TRANSPORT, 'TransportResult SdkTransport::command_head('
    )
    assert 'seed_head_weighted(positions, 1.0)' in mock_seed
    assert 'command_head_weighted(positions, 1.0)' in mock_command
    assert 'seed_head_weighted(positions, 1.0)' in sdk_seed
    assert 'command_head_weighted(positions, 1.0)' in sdk_command


def test_weighted_arm_api_validates_weight_and_preserves_cleanup_debt():
    publish = function_body(TRANSPORT, 'TransportResult publish_arm(')
    publish_validation = publish.index('valid_arm_weight(weight)')
    publish_weight = publish.index('arm_sdk->weight(weight)', publish_validation)
    publish_unlock = publish.index('unlockAndPublish()', publish_weight)
    assert publish_validation < publish_weight < publish_unlock

    seed = function_body(
        TRANSPORT, 'TransportResult SdkTransport::seed_head_weighted('
    )
    seed_validation = seed.index('valid_arm_weight(weight)')
    seed_debt = seed.index('head_maybe_active = true', seed_validation)
    seed_publish = seed.index('publish_arm(positions, sdk_weight)', seed_debt)
    seed_gate = seed.index('head_seeded = true', seed_publish)
    assert seed_validation < seed_debt < seed_publish < seed_gate

    command = function_body(
        TRANSPORT, 'TransportResult SdkTransport::command_head_weighted('
    )
    command_gate = command.index('if (!impl_->head_seeded)')
    command_validation = command.index('valid_arm_weight(weight)', command_gate)
    command_debt = command.index('head_maybe_active = true', command_validation)
    command_publish = command.index(
        'publish_arm(positions, sdk_weight)', command_debt
    )
    assert command_gate < command_validation < command_debt < command_publish

    hold = function_body(TRANSPORT, 'TransportResult SdkTransport::hold_head(')
    ambiguous_hold_gate = hold.index('head_weight_delivery_ambiguous')
    preserved_weight = hold.index(
        'command_head_weighted(positions, impl_->head_release_weight)',
        ambiguous_hold_gate,
    )
    assert ambiguous_hold_gate < preserved_weight
    assert 'command_head_weighted(positions, 1.0)' not in hold

    validator = function_body(TRANSPORT, 'bool valid_arm_weight(')
    assert 'std::isfinite(weight)' in validator
    assert 'weight >= 0.0' in validator
    assert 'weight <= 1.0' in validator

    release = function_body(
        TRANSPORT, 'TransportResult SdkTransport::release_head('
    )
    passive_branch = release.index('head_release_weight <= 0.0F')
    direct_zero = release.index('publish_zero_burst()', passive_branch)
    ramp = release.index('for (int step = 1;', direct_zero)
    assert passive_branch < direct_zero < ramp
    assert 'detail::descending_arm_release_weight(' in release[ramp:]


def test_arm_sdk_publish_unlocks_on_message_construction_exception():
    body = function_body(TRANSPORT, 'TransportResult publish_arm(')
    lock_mark = body.index('lock_held = true')
    indexed_write = body.index('motor_cmd().at(joint)', lock_mark)
    publish = body.index('unlockAndPublish()', indexed_write)
    exception_handler = body.index('catch (const std::exception', publish)
    recovery_unlock = body.index('arm_sdk->unlock()', exception_handler)
    assert lock_mark < indexed_write < publish < exception_handler < recovery_unlock
    assert 'ArmSdk frame exception:' in body[exception_handler:]


def test_required_services_and_transient_kill_are_present():
    for service in (
        '/r1/live_writer/prepare',
        '/r1/live_writer/stop',
        '/r1/live_writer/kill',
    ):
        assert service in NODE
    assert '/r1/live_writer/probe_head_ownership' in NODE
    assert '.transient_local()' in NODE
    assert '"head_ownership_probe_failed cause=deadman_released"' in NODE
    assert '"deadman_released", true)' in NODE
    assert 'release_head' in NODE
    assert 'kill_request_publisher_' in NODE
    assert 'kill_publisher_' not in NODE
    assert 'publish_kill_request(false)' in NODE


def test_launch_never_sets_live_environment_flags():
    assert 'SetEnvironmentVariable' not in LAUNCH
    assert 'ROBOT_ENABLE_ACTUATION' not in LAUNCH
    assert 'ROBOT_CONFIRM_OFF_CHARGER' not in LAUNCH


def test_prepare_enters_locomotion_after_stable_fsm_4():
    body = function_body(TRANSPORT, 'TransportResult SdkTransport::prepare(')
    stand_up = body.index('StandUp()')
    first_poll = body.index('GetFsmId', stand_up)
    assert first_poll > stand_up
    standing_id = re.search(
        r'constexpr\s+int\s+(\w*[Ss]tanding\w*)\s*=\s*4\s*;', body)
    assert re.search(r'fsm_id\s*==\s*4', body) or (
        standing_id is not None and
        re.search(rf'fsm_id\s*==\s*{standing_id.group(1)}', body))
    assert re.search(
        r'(?:\+\+\s*\w*stable\w*|\w*stable\w*\s*\+\+|'
        r'\w*stable\w*\s*\+=\s*1)', body, re.IGNORECASE)
    assert re.search(r'\w*stable\w*\s*=\s*0', body, re.IGNORECASE)
    assert re.search(
        r'\w*stable\w*\s*>=\s*(?:5|\w*stable\w*)', body, re.IGNORECASE)
    assert 'deadline' in body
    assert re.search(
        r'steady_clock::now\(\)\s*(?:>=|<)\s*\w*deadline\w*', body)
    assert 'sleep_for' in body or 'sleep_until' in body
    # A timeout must fail closed and include the last observable state.
    assert 'timeout' in body.lower()
    assert 'fsm' in body.lower()

    cancel_checks = [match.start() for match in re.finditer('should_cancel', body)]
    poll_loop = body.index('while (')
    assert 'using CancelCheck = std::function<bool()>' in TRANSPORT_HEADER
    assert any(check < stand_up for check in cancel_checks)
    assert any(poll_loop < check < first_poll for check in cancel_checks)
    assert any(check > first_poll for check in cancel_checks)

    start = body.index('Start()', first_poll)
    start_poll = body.index('wait_for_stable_fsm(kLocomotionFsmId', start)
    assert start > first_poll
    assert start_poll > start
    assert 'kLocomotionFsmId = 811' in body
    assert 'R1 Start error=' in body
    assert 'if (!enter_locomotion)' in body
    assert 'impl_->prepare_maybe_active = false' in body
    # A successful Start still owes StopMove cleanup until the writer is
    # explicitly stopped; only the head/arms-only branch may clear the flag.
    clear = body.index('impl_->prepare_maybe_active = false')
    assert clear > body.index('if (!enter_locomotion)')
    assert clear < start


def test_set_velocity_marks_potential_delivery_before_the_rpc():
    body = function_body(TRANSPORT, 'TransportResult SdkTransport::set_velocity(')
    rpc = body.index('SetVelocity(')
    assignment = re.search(r'velocity_maybe_active\s*=\s*true\s*;', body)
    assert assignment is not None
    assert assignment.start() < rpc
    # A nonzero/ambiguous RPC result must retain the flag for StopMove retry.
    assert not re.search(r'velocity_maybe_active\s*=\s*false', body)

    stop_body = function_body(TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()')
    stop_rpc = stop_body.index('StopMove()')
    clear = re.search(
        r'velocity_maybe_active\s*=\s*false\s*;', stop_body[stop_rpc:]
    )
    assert clear is not None

    destructor = function_body(TRANSPORT, 'SdkTransport::~SdkTransport()')
    assert 'velocity_maybe_active' in destructor
    assert 'stop_locomotion()' in destructor


def test_node_only_enters_sport_mode_for_locomotion_prepare():
    """FSM 811 entry is explicit and independent of joystick output."""
    prepare_call = NODE.index('return transport_->prepare(')
    snippet = NODE[prepare_call:prepare_call + 240]
    assert (
        '}, prepare_enter_locomotion_, static_prepare_mode_, prepare_speed_mode_);'
        in snippet
    )
    assert '}, enable_locomotion_);' not in snippet


def test_static_stand_retains_stopmove_cleanup_debt_until_reviewed_stop():
    prepare = function_body(
        TRANSPORT, 'TransportResult SdkTransport::prepare('
    )
    stand_up = prepare.index('StandUp()')
    no_locomotion = prepare.index('if (!enter_locomotion)', stand_up)
    static_branch = prepare.index(
        'if (!retain_stand_cleanup_debt)', no_locomotion
    )
    arms_only_clear = prepare.index(
        'impl_->prepare_maybe_active = false', static_branch
    )
    retained = prepare.index(
        'static Stand cleanup debt retained', arms_only_clear
    )
    assert stand_up < no_locomotion < static_branch < arms_only_clear < retained

    node_prepare = function_body(NODE, 'void on_prepare(')
    assert (
        'prepare_enter_locomotion_, static_prepare_mode_' in node_prepare
    )
    stop = function_body(
        TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()'
    )
    stop_move = stop.index('StopMove()')
    debt_clear = stop.index('impl_->prepare_maybe_active = false', stop_move)
    assert stop_move < debt_clear


def test_no_velocity_stop_fallback_requires_stable_entered_nonmoving_mode():
    """A rejected redundant zero is safe only before any velocity boundary."""
    prepare = function_body(
        TRANSPORT, 'TransportResult SdkTransport::prepare('
    )
    start = prepare.index('Start()')
    start_debt = prepare.rindex(
        'impl_->locomotion_mode_maybe_active = true', 0, start
    )
    assert start_debt < start

    stop = function_body(
        TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()'
    )
    stop_move = stop.index('StopMove()')
    fallback = stop.index(
        'detail::no_velocity_stop_fallback_eligible(', stop_move
    )
    get_fsm = stop.index('GetFsmId(', fallback)
    stable_gate = stop.index(
        'if (stable_samples == kRequiredStableSamples)', get_fsm
    )
    debt_clear = stop.index(
        'impl_->prepare_maybe_active = false', stable_gate
    )
    assert 'kRequiredStableSamples = 5' in stop[fallback:stable_gate]
    assert 'kMaxConfirmationPolls = 40' in stop[fallback:stable_gate]
    assert 'polls <= kMaxConfirmationPolls' in stop[fallback:stable_gate]
    assert 'detail::confirms_nonmoving_mode_sample(' in stop[
        fallback:stable_gate
    ]
    assert 'stable_samples = 0' in stop[get_fsm:stable_gate]
    assert 'nonmoving FSM confirmation timed out' in stop[stable_gate:]
    assert 'impl_->velocity_maybe_active' in stop[fallback:get_fsm]
    assert 'impl_->locomotion_mode_maybe_active' in stop[
        get_fsm:stable_gate
    ]
    assert stop_move < fallback < get_fsm < stable_gate < debt_clear

    # The ordinary accepted StopMove path clears every kind of debt.
    accepted = stop.index('if (result == 0)', stop_move)
    assert 'impl_->locomotion_mode_maybe_active = false' in stop[
        accepted:fallback
    ]
    assert 'impl_->velocity_maybe_active = false' in stop[accepted:fallback]
    # The fallback never clears velocity debt. It is unreachable once a
    # SetVelocity RPC boundary was crossed, even if delivery was ambiguous.
    assert 'impl_->velocity_maybe_active = false' not in stop[
        fallback:debt_clear + len('impl_->prepare_maybe_active = false')
    ]


def test_arms_running_prepare_keeps_velocity_pipeline_disabled():
    """The running arm stage may enter FSM 811, never SetVelocity."""
    live = (ROOT.parents[2] / 'scripts' / 'r1-live-session').read_text()
    stage = live[live.index('  arms-running)'):live.index('  head-probe)')]
    assert 'ENABLE_ARMS=true' in stage
    assert 'ENABLE_LOCOMOTION=false' in stage
    assert 'PREPARE_ENTER_LOCOMOTION=true' in stage
    assert 'set_velocity' not in stage
    assert 'enable_locomotion:="${ENABLE_LOCOMOTION}"' in live
    assert 'prepare_enter_locomotion:="${PREPARE_ENTER_LOCOMOTION}"' in live


def test_velocity_127_is_not_transport_success_or_enabled_by_legacy_switch():
    """The general transport remains fail-closed for every nonzero status."""
    live = (ROOT.parents[2] / 'scripts' / 'r1-live-session').read_text()
    assert 'unset R1_ALLOW_R1_VELOCITY_127_COMPAT' in live
    assert 'export R1_ALLOW_R1_VELOCITY_127_COMPAT' not in live
    assert 'VELOCITY_127_COMPAT=true' not in live

    velocity = function_body(
        TRANSPORT, 'TransportResult SdkTransport::set_velocity('
    )
    assert 'if (result == 0)' in velocity
    assert 'result == 127' not in velocity
    assert 'ambiguous nonzero status=' in velocity

    stop = function_body(
        TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()'
    )
    assert 'result == 127' not in stop
    assert 'velocity_127_compatibility' not in TRANSPORT


def test_velocity_127_probe_is_bounded_forward_only_and_opt_in():
    """One firmware experiment may admit 127 without weakening normal mode."""
    assert 'velocity_status_127_probe_enabled: false' in CONFIG
    assert (
        '"velocity_status_127_probe_enabled", false' in NODE
    )

    validation = function_body(NODE, 'void validate_parameters() const')
    for required in (
        'transport_name_ != "sdk"',
        '!enable_locomotion_',
        'enable_head_',
        'enable_arms_',
        '!prepare_enter_locomotion_',
        'exhibition_session_mode_',
        'velocity_limits_.forward > kVelocity127ProbeMaxForwardMps',
        '!velocity_127_probe_environment_confirmed()',
    ):
        assert required in validation

    velocity = function_body(NODE, 'void process_velocity(')
    assert 'kVelocity127ProbeMaxDurationSec = 1.5' in NODE
    assert 'kVelocity127ProbeMaxForwardMps = 0.15' in NODE
    assert 'std::clamp(shaped[0], 0.0, kVelocity127ProbeMaxForwardMps)' in velocity
    assert '0.0,\n        0.0};' in velocity
    assert 'age(send_now, velocity_127_probe_started_) >=' in velocity
    assert 'velocity_127_probe_window_complete' in velocity
    assert 'result.status_code == 127' in velocity
    assert 'locomotion_active_ = true' in velocity
    assert 'last_velocity_rpc_send_ = SteadyClock::now()' in velocity

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    assert 'velocity_127_probe_started_ = SteadyClock::time_point{}' in safe_stop

    watchdog = function_body(NODE, 'std::string active_watchdog_failure(')
    assert 'watchdog_velocity_127_probe_confirmation_lost' in watchdog

    environment = function_body(
        NODE, 'bool velocity_127_probe_environment_confirmed() const'
    )
    assert 'ROBOT_CONFIRM_UNITREE_EXPLORE_CLOSED' in environment
    assert 'ROBOT_CONFIRM_NO_PHONE_CONTROL' in environment

    for limit in ('max_forward_mps', 'max_lateral_mps', 'max_yaw_rps'):
        assert f"DeclareLaunchArgument('{limit}'" in LAUNCH
        assert f"LaunchConfiguration('{limit}')" in LAUNCH


def test_failed_stopmove_retries_are_bounded_and_debt_is_preserved():
    """An ambiguous stop must not hammer the R1 service at the 100 Hz tick rate."""
    safe_stop = function_body(NODE, 'void safe_stop_outputs(')

    assert 'kCleanupRetryIntervalSec{0.5}' in NODE
    assert 'last_locomotion_cleanup_attempt_' in safe_stop
    assert 'age(cleanup_now, last_locomotion_cleanup_attempt_) >=' in safe_stop
    assert 'locomotion_cleanup_pending = locomotion_active_' in safe_stop
    assert 'StopMove=retry_deferred' in safe_stop
    assert 'last_safe_stop_status_time_' in safe_stop

    velocity = function_body(NODE, 'void process_velocity(')
    velocity_call = velocity.index('transport_->set_velocity(')
    reset = velocity.rindex(
        'last_locomotion_cleanup_attempt_ = SteadyClock::time_point{}',
        0,
        velocity_call,
    )
    assert reset < velocity_call


def test_first_locomotion_command_is_logged_with_mode_and_result():
    """One short physical command must leave unambiguous evidence."""
    velocity = function_body(NODE, 'void process_velocity(')
    attempt = velocity.index('Locomotion command attempt mode=')
    call = velocity.index('transport_->set_velocity(', attempt)
    result = velocity.index('Locomotion command result ok=true', call)
    assert attempt < call < result
    assert 'Locomotion_command_failed mode=' in velocity[call:result]


def test_wireless_controller_mode_matches_captured_vendor_channel():
    assert 'locomotion_command_mode: "loco_rpc"' in CONFIG
    assert 'wireless_controller_rate_hz: 20.0' in CONFIG
    assert '"locomotion_command_mode", "loco_rpc"' in NODE
    assert 'std::make_unique<SdkTransport>(' in NODE
    assert 'locomotion_command_mode_ == "wireless_controller"' in NODE
    assert "DeclareLaunchArgument(\n            'wireless_controller_rate_hz'" in LAUNCH

    mapping = function_body(
        TRANSPORT, 'detail::velocity_to_wireless_controller('
    )
    assert 'lateral / kLateralCeilingMps' in mapping
    assert 'forward / kForwardCeilingMps' in mapping
    assert '-yaw / kYawCeilingRps' in mapping
    assert 'std::clamp' in mapping

    velocity = function_body(
        TRANSPORT, 'TransportResult SdkTransport::set_velocity('
    )
    assert 'rt/wirelesscontroller' in TRANSPORT
    assert 'velocity_to_wireless_controller(' in velocity
    assert 'message.keys(command.keys)' in velocity
    assert 'wireless_controller->Write(message)' in velocity

    stop = function_body(
        TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()'
    )
    assert 'kZeroFrames = 6' in stop
    assert 'milliseconds(50)' in stop
    assert 'wireless_controller->Write(zero)' in stop


def test_optional_speed_mode_is_bounded_to_the_isolated_probe():
    """The R1 speed-mode experiment must be explicit and happen before velocity."""
    prepare = function_body(TRANSPORT, 'TransportResult SdkTransport::prepare(')
    assert 'SetSpeedMode(speed_mode)' in prepare
    assert 'R1 SetSpeedMode(' in prepare
    speed_call = prepare.index('SetSpeedMode(speed_mode)')
    final_success = prepare.index('stand_up_state.detail + "; " + locomotion_state.detail')
    assert speed_call < final_success

    validation = function_body(NODE, 'void validate_parameters() const')
    assert 'prepare_speed_mode must be -1 (disabled) or 1' in validation
    assert 'prepare_speed_mode is admitted only by the isolated legs-only status-127 probe' in validation

    assert "DeclareLaunchArgument('prepare_speed_mode'" in LAUNCH
    assert "value_type=int" in LAUNCH


def test_velocity_disabled_811_allows_only_one_upper_body_path():
    validation = function_body(NODE, 'void validate_parameters() const')
    gate = validation.index(
        'if (prepare_enter_locomotion_ && !enable_locomotion_)'
    )
    snippet = validation[gate:gate + 520]
    assert 'enable_arms_ && !enable_head_' in snippet
    assert 'enable_head_ && !enable_arms_' in snippet
    assert 'if (!arms_only && !head_only)' in snippet
    assert 'exactly one upper-body commissioning path' in snippet


def test_arm_sdk_release_is_a_bounded_approximately_one_second_ramp():
    body = function_body(TRANSPORT, 'TransportResult SdkTransport::release_head(')
    assert 'deadline' in body
    assert re.search(r'milliseconds\s*\(\s*(?:12[0-9][0-9]|13[0-9][0-9]|1400)\s*\)', body)
    assert re.search(r'(?:milliseconds\s*\(\s*1000\s*\)|seconds\s*\(\s*1\s*\))', body)
    assert re.search(r'milliseconds\s*\(\s*10\s*\)', body)
    assert re.search(r'(?:release_)?steps?\s*=\s*100', body, re.IGNORECASE)
    assert re.search(r'zero_?burst_?frames?\s*=\s*30', body, re.IGNORECASE)
    assert re.search(r'milliseconds\s*\(\s*3100\s*\)', body)
    assert re.search(
        r'for\s*\(\s*int\s+frame\s*=\s*0\s*;\s*'
        r'frame\s*<\s*kZeroBurstFrames', body)
    assert 'sleep_until(consume_until)' in body
    burst_definition = body.index('const auto publish_zero_burst')
    burst_loop = body.index('for (int frame = 0;', burst_definition)
    burst_publish = body.index('publish_arm(positions, 0.0F)', burst_loop)
    burst_zero_state = body.index('head_release_weight = 0.0F', burst_publish)
    assert 'head_weight_delivery_ambiguous = true' in body[
        burst_definition:burst_loop]
    assert burst_loop < burst_publish < burst_zero_state
    assert re.search(r'publish_arm\s*\(\s*positions\s*,\s*weight\s*\)', body)
    assert re.search(r'publish_arm\s*\(\s*positions\s*,\s*0\.0F?\s*\)', body)
    assert body.count('publish_zero_burst()') >= 2
    assert re.search(r'steady_clock::now\(\)\s*>=\s*\w*deadline\w*', body)
    ramp_start = body.index(
        'const float release_start_weight = impl_->head_release_weight')
    ramp_publish = body.index('publish_arm(positions, weight)', ramp_start)
    ramp_failure = body.index('if (!ramp_result.ok)', ramp_publish)
    ramp_weight_update = body.index('head_release_weight = weight', ramp_failure)
    assert 'head_weight_delivery_ambiguous = true' in body[
        ramp_failure:ramp_weight_update]
    assert ramp_start < ramp_publish < ramp_failure < ramp_weight_update
    final_zero = body.index('const TransportResult final_zero')
    zero_burst = body.index('const TransportResult zero_burst', final_zero)
    burst_failure = body.index('if (!zero_burst.ok)', zero_burst)
    degraded = body.index('if (!ramp_result.ok)', burst_failure)
    seeded_clear = body.index('head_seeded = false', degraded)
    degraded_success = body.index('return success(', seeded_clear)
    assert final_zero < zero_burst < burst_failure < degraded
    assert degraded < seeded_clear < degraded_success
    assert 'after degraded ramp' in body[degraded_success:]

    direct_result = body.index('const TransportResult direct_zero')
    direct_failure = body.index('if (!direct_zero.ok)', direct_result)
    direct_clear = body.index('head_seeded = false', direct_failure)
    assert 'head_seeded = false' not in body[direct_result:direct_failure]
    assert direct_result < direct_failure < direct_clear

    burst_result = body.index('const TransportResult zero_burst', final_zero)
    burst_failure = body.index('if (!zero_burst.ok)', burst_result)
    first_normal_clear = body.index('head_seeded = false', burst_failure)
    assert 'head_seeded = false' not in body[burst_result:burst_failure]
    assert burst_result < burst_failure < first_normal_clear


def test_ambiguous_first_arm_seed_owes_direct_zero_cleanup():
    seed = function_body(
        TRANSPORT, 'TransportResult SdkTransport::seed_head_weighted('
    )
    debt = seed.index('head_maybe_active = true')
    publish = seed.index('publish_arm(positions, sdk_weight)', debt)
    assert debt < publish
    assert 'head_maybe_active = false' not in seed[publish:]

    release = function_body(
        TRANSPORT, 'TransportResult SdkTransport::release_head('
    )
    ambiguous = release.index('if (!impl_->head_seeded')
    direct_zero = release.index('publish_zero_burst()', ambiguous)
    ramp = release.index('for (int step = 1;', direct_zero)
    assert ambiguous < direct_zero < ramp
    assert 'head_maybe_active = false' in release[direct_zero:ramp]

    destructor = function_body(TRANSPORT, 'SdkTransport::~SdkTransport()')
    assert 'impl_->head_seeded || impl_->head_maybe_active' in destructor

    normal_head = function_body(NODE, 'void process_head(')
    normal_claim = normal_head.index('head_claimed_ = true')
    normal_seed = normal_head.index('transport_->seed_head(', normal_claim)
    assert normal_claim < normal_seed

    recenter = function_body(NODE, 'void process_head_recenter(')
    recenter_claim = recenter.index('head_claimed_ = true')
    recenter_seed = recenter.index(
        'transport_->seed_head_weighted(', recenter_claim)
    assert recenter_claim < recenter_seed


def test_node_enforces_post_prepare_rearm_and_combined_command_barrier():
    completion = function_body(NODE, 'void finish_prepare_if_ready()')
    rearm_start = completion.index(
        'prepare_rearm_gate_.prepared(enable_locomotion_)')
    freshness_clear = completion.index('clear_command_freshness()', rearm_start)
    assert freshness_clear > rearm_start

    deadman = function_body(NODE, 'void on_deadman(')
    assert 'prepare_rearm_gate_.observe_deadman(deadman_active_)' in deadman
    assert 'clear_command_freshness()' in deadman

    velocity = function_body(NODE, 'void on_velocity(')
    assert 'prepare_rearm_gate_.observe_velocity(latest_velocity_)' in velocity

    authorization = function_body(NODE, 'AuthorizationInput authorization_input(')
    assert 'result.prepared = prepared_ && prepare_rearm_gate_.ready()' in authorization

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    assert 'prepare_rearm_gate_.reset()' in safe_stop
    assert 'clear_command_freshness()' in safe_stop
    assert re.search(
        r'if\s*\(\s*enable_locomotion_\s*\|\|\s*enable_prepare_\s*\)',
        safe_stop,
    )
    assert not re.search(r'if\s*\(\s*locomotion_active_\s*\)', safe_stop)

    armed = function_body(NODE, 'void publish_armed()')
    assert 'prepared_ && prepare_rearm_gate_.ready()' in armed
    assert 'deadman_fresh(now)' in armed
    assert 'state_fresh(now)' in armed
    assert 'enabled_commands_fresh(now)' in armed
    assert 'prepare_rearm=' in NODE

    readiness = function_body(NODE, 'bool enabled_commands_fresh(')
    assert '!enable_head_ || head_fresh(now)' in readiness
    assert '!enable_arms_ || arm_fresh(now)' in readiness
    assert '!enable_locomotion_ || velocity_fresh(now)' in readiness
    assert NODE.count('const auto authorization_now = SteadyClock::now();') == 3
    assert NODE.count('enabled_commands_fresh(authorization_now)') == 3


def test_node_validates_both_sources_against_the_same_explicit_transport():
    validation = function_body(NODE, 'bool vr_source_ok(')
    assert validation.count('valid_vr_source(') == 2
    assert 'environment.vr_transport == vr_transport_' in validation
    assert 'valid_vr_source(expected_vr_source_ip_, vr_transport_)' in validation
    assert 'valid_vr_source(environment.vr_source_ip, environment.vr_transport)' in validation
    assert 'valid_unicast_ipv4(' not in NODE


def test_prepare_service_dispatches_async_and_returns_without_waiting():
    prepare = function_body(NODE, 'void on_prepare(')
    progress_guard = prepare.index('if (prepare_in_progress_)')
    async_launch = prepare.index('std::async(std::launch::async')
    transport_prepare = prepare.index('transport_->prepare(', async_launch)
    progress_set = prepare.index('prepare_in_progress_ = true')

    # Ownership must be visible before std::async can start using LocoClient;
    # otherwise stop/watchdog paths have a race in which they may enter the
    # transport concurrently with the worker.
    assert progress_guard < progress_set < async_launch < transport_prepare
    assert 'prepare_cancel_requested_.store(false)' in prepare[:async_launch]
    assert 'prepare_cancel_requested_.load()' in prepare[async_launch:]
    assert 'prepare_in_progress_ = false' in prepare[transport_prepare:]
    assert 'prepare_future_.wait(' not in prepare
    assert 'prepare_future_.wait_for(' not in prepare
    assert 'prepare_future_.get(' not in prepare
    assert 'prepare started asynchronously' in prepare[progress_set:]


def test_timer_polls_prepare_future_nonblocking_and_reauthorizes_completion():
    timer = function_body(NODE, 'void on_timer()')
    finish_call = timer.index('finish_prepare_if_ready()')
    progress_branch = timer.index('if (prepare_in_progress_)', finish_call)
    branch_return = timer.index('return;', progress_branch)
    first_output_path = min(timer.index('process_head('), timer.index('process_velocity('))
    assert finish_call < progress_branch < branch_return < first_output_path

    completion = function_body(NODE, 'void finish_prepare_if_ready()')
    ready_poll = completion.index('prepare_future_.wait_for(std::chrono::seconds(0))')
    ready_status = completion.index('std::future_status::ready', ready_poll)
    future_get = completion.index('prepare_future_.get()', ready_status)
    cancel_exchange = completion.index(
        'prepare_cancel_requested_.exchange(false)', future_get)
    completion_gate = completion.index('authorize_live_send(', cancel_exchange)
    prepared = completion.index('prepared_ = true', completion_gate)
    rearm = completion.index(
        'prepare_rearm_gate_.prepared(enable_locomotion_)', prepared)
    assert ready_poll < ready_status < future_get < cancel_exchange
    assert cancel_exchange < completion_gate < prepared < rearm
    assert 'prepare_future_.wait()' not in completion
    assert 'prepare_completion_gate_failed:' in completion


def test_prepared_static_and_exhibition_sessions_keep_watchdog_active():
    timer = function_body(NODE, 'void on_timer()')
    watchdog_gate = timer.index('const bool safety_watchdog_required =')
    prepared_watchdog = timer.index(
        '(prepared_ && (static_prepare_mode_ || exhibition_session_mode_))',
        watchdog_gate,
    )
    watchdog_call = timer.index(
        'active_watchdog_failure(now)', prepared_watchdog
    )
    fail_closed = timer.index('safe_stop_outputs(', watchdog_call)
    assert watchdog_gate < prepared_watchdog < watchdog_call < fail_closed

    watchdog = function_body(
        NODE, 'std::string active_watchdog_failure('
    )
    for required_gate in (
        'kill_clear_fresh(now)',
        'state_fresh(now)',
        'motor_health_fresh(now)',
        'live_environment_complete(environment)',
        'commissioning_parameter_ok(environment)',
        'control_source_ok(environment)',
        'profile_ != "slow-safe"',
    ):
        assert required_gate in watchdog
    assert 'if (static_prepare_mode_)' in watchdog


def test_exhibition_arm_hold_preserves_pending_feedback_follow_guard():
    """A VR dropout must not erase evidence that the last arm target stalled."""
    timer = function_body(NODE, 'void on_timer()')
    dropout = timer[timer.index('const bool exhibition_dropout'):]
    dropout = dropout[:dropout.index('const std::string arm_follow_reason')]
    assert 'clear_arm_feedback_follow()' not in dropout

    hold = function_body(NODE, 'void maintain_exhibition_arm_hold(')
    assert 'transport_->hold_head(held_head_target_)' in hold
    assert 'clear_arm_feedback_follow()' not in hold


def test_prepare_watchdog_cancels_without_concurrent_transport_calls():
    watchdog = function_body(NODE, 'std::string prepare_watchdog_failure(')
    for required_gate in (
        'kill_clear_fresh(now)',
        'deadman_fresh(now)',
        'state_fresh(now)',
        'commissioning_parameter_ok(environment)',
        'control_source_ok(environment)',
        'profile_ != "slow-safe"',
    ):
        assert required_gate in watchdog

    timer = function_body(NODE, 'void on_timer()')
    assert 'prepare_watchdog_failure(now)' in timer
    assert 'safe_stop_outputs("prepare_" + failure, true)' in timer

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    cancel = safe_stop.index('prepare_cancel_requested_.store(true)')
    deferred_transport = safe_stop.index('!prepare_in_progress_', cancel)
    first_transport_call = min(
        safe_stop.index('transport_->stop_locomotion()', deferred_transport),
        safe_stop.index('transport_->hold_head(', deferred_transport),
    )
    assert cancel < deferred_transport < first_transport_call
    assert 'locomotion_cleanup_pending = !stopped.ok' in safe_stop
    assert 'head_cleanup_pending = !released.ok' in safe_stop
    assert 'locomotion_active_ = locomotion_cleanup_pending' in safe_stop
    assert 'head_claimed_ = head_cleanup_pending' in safe_stop

    completion = function_body(NODE, 'void finish_prepare_if_ready()')
    joined = completion.index('prepare_future_.get()')
    progress_clear = completion.index('prepare_in_progress_ = false', joined)
    cancelled = completion.index('if (cancelled)', progress_clear)
    deferred_cleanup = completion.index(
        'safe_stop_outputs("prepare_cancelled result="', cancelled)
    assert joined < progress_clear < cancelled < deferred_cleanup

    deadman = function_body(NODE, 'void on_deadman(')
    assert 'outputs_active() || prepare_in_progress_' in deadman

    armed = function_body(NODE, 'void publish_armed()')
    assert '!prepare_in_progress_' in armed
    assert 'prepare_in_progress=' in NODE

    destructor = function_body(NODE, '~R1LiveWriter() override')
    shutdown_cancel = destructor.index('prepare_cancel_requested_.store(true)')
    shutdown_wait = destructor.index('prepare_future_.wait()', shutdown_cancel)
    shutdown_stop = destructor.index('safe_stop_outputs("node_shutdown", false)', shutdown_wait)
    assert shutdown_cancel < shutdown_wait < shutdown_stop


def test_stop_and_kill_services_do_not_claim_success_while_cleanup_is_pending():
    stop = function_body(NODE, 'void on_stop(')
    kill = function_body(NODE, 'void on_kill_service(')
    for body in (stop, kill):
        safe_stop = body.index('safe_stop_outputs(')
        truthful_response = body.index(
            'response->success = !outputs_active() && !prepare_in_progress_',
            safe_stop,
        )
        assert truthful_response > safe_stop
        assert 'physical E-stop' in body[truthful_response:]


def test_live_send_cannot_bypass_prepare_and_rearm():
    validation = function_body(NODE, 'void validate_parameters() const')
    assert re.search(
        r'send_commands_\s*&&\s*\(\s*!enable_prepare_\s*\|\|\s*'
        r'!require_prepare_\s*\)',
        validation,
    )
    assert 'send_commands=true requires enable_prepare=true and require_prepare=true' \
        in validation


def test_every_first_sdk_action_is_reauthorized_after_transport_initialization():
    prepare = function_body(NODE, 'void on_prepare(')
    prepare_initialize = prepare.index('ensure_transport()')
    prepare_post_gate = prepare.index(
        'const GateDecision post_initialize_gate = authorize_live_send(',
        prepare_initialize,
    )
    prepare_worker = prepare.index('std::async(std::launch::async', prepare_post_gate)
    assert prepare_initialize < prepare_post_gate < prepare_worker

    head = function_body(NODE, 'void process_head(')
    head_initialize = head.index('ensure_transport()')
    head_post_gate = head.index(
        'const GateDecision send_gate = authorize_live_send(', head_initialize)
    head_seed = head.index('transport_->seed_head(', head_post_gate)
    head_command = head.index('transport_->command_head(', head_post_gate)
    assert head_initialize < head_post_gate < head_seed < head_command
    assert 'enabled_commands_fresh(send_now)' in head[head_post_gate:head_seed]

    seed_extract = head.index('const std::array<double, 2> seed{', head_post_gate)
    absolute_check = head.index(
        'head_within_absolute_limits(seed, head_absolute_limits_)',
        seed_extract,
    )
    seed_window = head.index('head_seed_requires_explicit_recenter', absolute_check)
    stable_feedback = head.index(
        'head_seed_waiting_for_stable_feedback', seed_window)
    stable_timeout = head.index(
        'head_seed_stable_feedback_timeout', stable_feedback)
    stable_samples = head.index(
        'head_seed_settle_stable_samples_ < kHeadSeedStableSamples',
        stable_timeout,
    )
    assert head_post_gate < seed_extract < absolute_check
    assert (
        absolute_check < seed_window < stable_feedback < stable_timeout
        < stable_samples < head_seed
    )
    settle_path = head[stable_feedback:head_seed]
    assert 'state_sequence_ != head_seed_settle_last_state_sequence_' in settle_path
    assert 'head_velocity_valid_' in settle_path
    assert 'reset_head_seed_settle()' in settle_path

    held_seed = head.index('held_head_target_ = latest_state_', stable_samples)
    assert stable_samples < held_seed < head_seed

    target = head.index('absolute_head_target(', head_seed)
    command = head.index('transport_->command_head(', target)
    effective_delta = head.index('effective_delta', target)
    anti_windup = head.index('last_head_yaw_pitch_ = effective_delta', command)
    assert head_seed < target < effective_delta < command < anti_windup
    command_path = head[held_seed:command]
    assert 'held_head_target_ = latest_state_' in command_path
    assert command_path.count('held_head_target_[') == 2
    assert 'held_head_target_[kHeadPitchIndex]' in command_path
    assert 'held_head_target_[kHeadYawIndex]' in command_path
    assert '-head_limits_.yaw' not in head
    assert '-head_limits_.pitch' not in head

    velocity = function_body(NODE, 'void process_velocity(')
    velocity_initialize = velocity.index('ensure_transport()')
    velocity_post_gate = velocity.index(
        'const GateDecision send_gate = authorize_live_send(', velocity_initialize)
    velocity_send = velocity.index('transport_->set_velocity(', velocity_post_gate)
    assert velocity_initialize < velocity_post_gate < velocity_send
    assert 'enabled_commands_fresh(send_now)' in velocity[
        velocity_post_gate:velocity_send]


def test_head_feedback_and_limits_use_unambiguous_immutable_conventions():
    joint_state = function_body(NODE, 'void on_joint_state(')
    assert 'message->name.size() != message->velocity.size()' in joint_state
    assert 'candidate[kHeadYawIndex], candidate[kHeadPitchIndex]' in joint_state
    assert '!static_prepare_mode_ &&' in joint_state
    assert '!prepare_in_progress_ &&' in joint_state
    assert '!head_within_feedback_envelope(candidate_head, head_absolute_limits_)' \
        in joint_state
    assert 'joint_state_rejected=head_outside_official_absolute_envelope_plus_feedback_margin' \
        in joint_state
    assert 'kHeadFeedbackEnvelopeToleranceRad = 0.01' in NODE
    deadman = function_body(NODE, 'void on_deadman(')
    assert 'became_ready = !exhibition_session_mode_ &&' in deadman
    assert 'prepare_rearm_gate_.observe_deadman(deadman_active_)' in deadman
    session = function_body(NODE, 'void on_session_armed(')
    assert 'safe_stop_outputs("exhibition_session_disarmed", true)' in session

    normal_head = function_body(NODE, 'void process_head(')
    envelope = normal_head.index('!head_within_absolute_limits(seed, head_absolute_limits_)')
    claim = normal_head.index('transport_->seed_head(held_head_target_)', envelope)
    assert 'safe_stop_outputs(' in normal_head[envelope:claim]
    stable = normal_head.index('head_seed_settle_stable_samples_ < kHeadSeedStableSamples')
    seeded = normal_head.index('transport_->seed_head(held_head_target_)', stable)
    verified = normal_head.index('head_tracking_seed_verified_ = true', seeded)
    assert stable < seeded < verified
    assert 'head_tracking_seed_verified_ = false' in function_body(
        NODE, 'void reset_head_seed_settle()'
    )
    assert 'begin_automatic_exhibition_head_center(send_now, seed)' in normal_head
    assert 'exhibition_session_mode_' in normal_head
    auto_center = function_body(
        NODE, 'void process_automatic_exhibition_head_center('
    )
    assert 'clamp_head_to_absolute_envelope(seed, head_absolute_limits_)' \
        in auto_center
    assert 'transport_->seed_head(held_head_target_)' in auto_center
    assert 'step_head_toward_zero(' in auto_center
    assert 'head_recenter_follow_timeout_sec_' in auto_center
    assert 'head_recenter_confirmation_samples_ >= 5' in auto_center
    assert 'head_auto_center=complete' in auto_center
    assert '!enabled_commands_fresh(now)' in auto_center
    pause = auto_center.index('head_auto_center=paused reason=vr_reconnecting')
    resume = auto_center.index('head_auto_center=resumed reason=vr_recovered')
    assert pause < resume
    assert 'maintain_exhibition_arm_hold("head_auto_center_vr_reconnecting")' \
        in auto_center[pause:resume]
    assert 'maintain_exhibition_velocity_zero(' \
        in auto_center[pause:resume]
    assert 'head_recenter_last_advance_time_ = now' \
        in auto_center[pause:resume]
    assert 'head_recenter_follow_wait_since_ = SteadyClock::time_point{}' \
        in auto_center[pause:resume]
    pause_branch = auto_center[pause:resume]
    assert 'safe_stop_outputs(' not in pause_branch
    assert 'head_auto_center_paused=' in NODE
    assert 'safe_stop_outputs(' not in auto_center[
        auto_center.index('head_recenter_confirmation_samples_ >= 5'):
    ]

    timer = function_body(NODE, 'void on_timer()')
    center = timer.index('process_automatic_exhibition_head_center(now, dt)')
    arms = timer.index('process_arms(now, dt)')
    locomotion = timer.index('process_velocity(now, dt)')
    assert center < arms < locomotion
    assert 'enable_arms_ && upper_allowed && !automatic_head_center_active_' in timer
    assert 'enable_locomotion_ && !automatic_head_center_active_' in timer

    safe_stop = function_body(NODE, 'void safe_stop_outputs(')
    assert 'clamp_head_to_absolute_envelope(' in safe_stop

    assert 'const HeadAbsoluteLimits head_absolute_limits_{};' in NODE
    for forbidden_parameter in (
        'head_yaw_min_rad', 'head_yaw_max_rad',
        'head_pitch_min_rad', 'head_pitch_max_rad',
    ):
        assert forbidden_parameter not in NODE
        assert forbidden_parameter not in CONFIG
        assert forbidden_parameter not in LAUNCH

    for relative_parameter in (
        'max_head_yaw_delta_rad',
        'max_head_pitch_delta_rad',
        'max_head_yaw_delta_rate_rad_s',
        'max_head_pitch_delta_rate_rad_s',
    ):
        assert relative_parameter in NODE
        assert relative_parameter in CONFIG

    assert 'max_head_yaw_rad' not in NODE
    assert 'max_head_pitch_rad' not in NODE


def test_head_probe_requires_new_dedicated_ack_and_checks_feedback_velocity():
    validation = function_body(NODE, 'void validate_parameters() const')
    assert 'head_ownership_probe_environment_confirmed()' in validation
    assert 'ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1' in validation
    assert 'legacy recenter' in validation
    assert 'head_ownership_probe_only=true' in validation

    legacy_service = function_body(NODE, 'void on_recenter_head(')
    assert 'full physical recenter is suspended' in legacy_service
    assert '/r1/live_writer/probe_head_ownership' in legacy_service

    recenter = function_body(NODE, 'void process_head_recenter(')
    feedback_helper = function_body(NODE, 'std::string head_probe_feedback_failure(')
    assert feedback_helper.count(
        'head_probe_max_feedback_velocity_rad_s_') >= 2
    assert 'head_recenter_max_feedback_velocity_rad_s_' not in feedback_helper
    seed = recenter.index('transport_->seed_head_weighted(')
    zero_weight = recenter.index('head_probe_zero_weight_hold_sec_', seed)
    ramp = recenter.index('head_probe_weight_ramp_sec_', zero_weight)
    command = recenter.index('publish_head_probe_frame(1.0)', ramp)
    assert seed < zero_weight < ramp < command
    assert 'head_recenter_max_feedback_velocity_rad_s: 0.12' in CONFIG
    assert 'head_probe_max_feedback_velocity_rad_s: 0.04' in CONFIG
    assert 'head_probe_seed_stability_rad: 0.003' in CONFIG
    assert 'head_probe_seed_samples: 10' in CONFIG

    pending = recenter[
        recenter.index('HeadRecenterState::PendingFreshSeed'):
        recenter.index('const TransportResult initialized = ensure_transport()')
    ]
    position_check = pending.index('head_probe_seed_candidate_')
    transport_boundary = recenter.index(
        'const TransportResult initialized = ensure_transport()'
    )
    assert position_check < transport_boundary
    assert 'head_probe_seed_stability_rad_' in pending
    assert 'head_probe_seed_samples_' in pending


def test_head_probe_keeps_actionable_weight_target_feedback_diagnostics():
    diagnostics = function_body(NODE, 'std::string head_probe_diagnostics(')
    for field in (
        'weight=',
        'feedback_yaw=', 'feedback_pitch=',
        'target_yaw=', 'target_pitch=',
        'error_yaw=', 'error_pitch=', 'progress_yaw=',
        'max_other_joint=', 'max_other_delta=',
    ):
        assert field in diagnostics

    recenter = function_body(NODE, 'void process_head_recenter(')
    assert 'outbound_feedback_timeout' in recenter
    assert 'return_feedback_timeout' in recenter
    assert 'head_ownership_probe_passed' not in recenter
    assert 'head_ownership_probe=passed terminal=true' in recenter


def test_supervisor_kill_ack_does_not_erase_primary_fail_closed_reason():
    on_kill = function_body(NODE, 'void on_kill(')
    assert 'const bool was_locally_latched = local_kill_latched_' in on_kill
    assert 'const bool cleanup_retry_due = !prepare_in_progress_ && outputs_active()' \
        in on_kill
    guarded_stop = on_kill.index(
        'if (!was_locally_latched || cleanup_retry_due)')
    stop = on_kill.index('safe_stop_outputs(', guarded_stop)
    assert guarded_stop < stop
    assert '"kill_signal_active", true, false)' in on_kill[stop:]


def test_ambiguous_prepare_is_tracked_until_stable_or_stopped():
    prepare = function_body(TRANSPORT, 'TransportResult SdkTransport::prepare(')
    stand_up = prepare.index('StandUp()')
    mark = prepare.index('prepare_maybe_active = true')
    stable_clear = prepare.index('prepare_maybe_active = false', stand_up)
    stable_success = prepare.index('return success(', stable_clear)
    assert mark < stand_up < stable_clear < stable_success

    stop = function_body(TRANSPORT, 'TransportResult SdkTransport::stop_locomotion()')
    stop_rpc = stop.index('StopMove()')
    prepare_clear = stop.index('prepare_maybe_active = false', stop_rpc)
    assert prepare_clear > stop_rpc

    destructor = function_body(TRANSPORT, 'SdkTransport::~SdkTransport()')
    assert 'prepare_maybe_active || impl_->velocity_maybe_active' in destructor
