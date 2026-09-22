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
    assert "default_value='mock'" in LAUNCH
    assert "default_value='false'" in LAUNCH


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
    clear = re.search(r'velocity_maybe_active\s*=\s*false\s*;', stop_body)
    assert clear is not None
    assert clear.start() > stop_rpc

    destructor = function_body(TRANSPORT, 'SdkTransport::~SdkTransport()')
    assert 'velocity_maybe_active' in destructor
    assert 'stop_locomotion()' in destructor


def test_node_only_enters_sport_mode_for_locomotion_prepare():
    """Head/arms-only prepare must stop at stable FSM 4."""
    prepare_call = NODE.index('return transport_->prepare(')
    assert '}, enable_locomotion_);' in NODE[prepare_call:prepare_call + 220]


def test_arm_sdk_release_is_a_bounded_approximately_one_second_ramp():
    body = function_body(TRANSPORT, 'TransportResult SdkTransport::release_head(')
    assert 'deadline' in body
    assert re.search(r'milliseconds\s*\(\s*(?:12[0-9][0-9]|13[0-9][0-9]|1400)\s*\)', body)
    assert re.search(r'(?:milliseconds\s*\(\s*1000\s*\)|seconds\s*\(\s*1\s*\))', body)
    assert re.search(r'milliseconds\s*\(\s*20\s*\)', body)
    assert re.search(r'(?:release_)?steps?\s*=\s*50', body, re.IGNORECASE)
    assert re.search(r'zero_?burst_?frames?\s*=\s*15', body, re.IGNORECASE)
    assert re.search(r'milliseconds\s*\(\s*2700\s*\)', body)
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


def test_node_uses_private_unicast_validation_for_both_vr_source_values():
    validation = function_body(NODE, 'bool vr_source_ok(')
    assert validation.count('valid_private_unicast_ipv4(') == 2
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


def test_prepare_watchdog_cancels_without_concurrent_transport_calls():
    watchdog = function_body(NODE, 'std::string prepare_watchdog_failure(')
    for required_gate in (
        'kill_clear_fresh(now)',
        'deadman_fresh(now)',
        'state_fresh(now)',
        'commissioning_parameter_ok(environment)',
        'vr_source_ok(environment)',
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
    stable_feedback = head.index('head_seed_requires_stable_feedback', seed_window)
    assert head_post_gate < seed_extract < absolute_check
    assert absolute_check < seed_window < stable_feedback < head_seed

    held_seed = head.index('held_head_target_ = latest_state_', stable_feedback)
    assert stable_feedback < held_seed < head_seed

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
    assert 'head_within_absolute_limits(candidate_head, head_absolute_limits_)' \
        in joint_state
    assert 'joint_state_rejected=head_outside_official_absolute_envelope' \
        in joint_state

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
