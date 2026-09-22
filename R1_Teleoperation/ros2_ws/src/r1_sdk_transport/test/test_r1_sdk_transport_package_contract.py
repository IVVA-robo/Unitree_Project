"""Static contract checks for the SDK transport safety boundary."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src' / 'r1_sdk_transport.cpp').read_text()
OBSERVER = (PACKAGE / 'src' / 'r1_arm_sdk_traffic_observer.cpp').read_text()
CONFIG = (PACKAGE / 'config' / 'r1_sdk_transport.yaml').read_text()
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text()
LAUNCH = (PACKAGE / 'launch' / 'r1_sdk_transport.launch.py').read_text()
CODE = '\n'.join(line.split('//', 1)[0] for line in SOURCE.splitlines())
PREFLIGHT = PACKAGE.parents[2] / 'scripts' / 'r1-sdk-preflight'
OBSERVER_WRAPPER = PACKAGE.parents[2] / 'scripts' / 'r1-arm-sdk-traffic-check'
OFFLINE_CHECK = PACKAGE.parents[2] / 'scripts' / 'r1-offline-commissioning-check'


def test_writer_channels_are_not_created():
    """The read-only transport must not instantiate command writers/clients."""
    assert 'ArmSdk' not in CODE
    assert 'LocoClient' not in CODE
    assert 'CreateSendChannel' not in CODE
    assert 'ChannelPublisher' not in CODE
    assert 'LowCmd' not in CODE


def test_arm_sdk_traffic_observer_is_a_bounded_subscriber_only():
    """The separate traffic gate may deserialize LowCmd but never write it."""
    code = '\n'.join(line.split('//', 1)[0] for line in OBSERVER.splitlines())
    assert 'CreateRecvChannel<unitree_hg::msg::dds_::LowCmd_>' in code
    assert 'CreateRecvChannel<std_msgs::msg::dds_::String_>' in code
    assert 'command_channel->GetReader()->GetNative()' in code
    assert 'ChannelFactory::Instance()->Init' in code
    assert '"rt/arm_sdk"' in code
    for forbidden in (
        'ChannelPublisher', 'ChannelSubscriber', 'CreateSendChannel', 'LocoClient',
        'publisher::ArmSdk', 'Write(', 'unlockAndPublish',
    ):
        assert forbidden not in code
    assert 'std::this_thread::sleep_until' in code
    assert 'kExitActiveFresh = 10' in OBSERVER
    assert 'kExitTrafficObserved = 11' in OBSERVER
    assert 'kExitActionStateMissing = 12' in OBSERVER
    assert 'kExitActionStateMalformed = 13' in OBSERVER
    assert 'kExitActionStateActive = 14' in OBSERVER
    assert 'kExitMatchEvidenceFailed = 15' in OBSERVER
    # The vendor async BlockQueue may discard an accepted sample during
    # CloseChannel teardown.  This gate must process both streams synchronously
    # so "any LowCmd sample blocks" remains true at the window boundary.
    assert OBSERVER.count('}, 0);') == 2
    assert '}, 10);' not in OBSERVER


def test_arm_sdk_observer_requires_stable_typed_writer_match():
    """Exit zero must be impossible for an unmatched or churning reader."""
    for token in (
        'std::uint64_t expected_writers{1}',
        '--expected-writers must be 1 or 2',
        'kRequiredMatchStableSec = 2.0',
        'dds_get_subscription_matched_status',
        'dds_get_matched_publications',
        'dds_get_matched_publication_data',
        'publication->topic_name',
        'publication->type_name',
        'status.current_count_change != 0',
        'status.total_count_change != 0',
        'current_handles != evidence.baseline_handles',
        'ARM_SDK_MATCH status=',
        'if (!match.accepted)',
        'return kExitMatchEvidenceFailed;',
    ):
        assert token in OBSERVER
    assert OBSERVER.index('if (!match.accepted)') < OBSERVER.index(
        'return action_stale ? kExitActionStateMissing : kExitClear;')


def test_physical_boundaries_pin_phase_specific_writer_counts():
    """Pre-init gates expect one writer; the post-init probe expects two."""
    preflight = PREFLIGHT.read_text()
    prepare = (PACKAGE.parents[2] / 'scripts' / 'r1-robot-prepare').read_text()
    probe = (
        PACKAGE.parents[2] / 'scripts' / 'r1-robot-head-ownership-probe'
    ).read_text()
    assert '--expected-writers 1' in preflight
    assert '--expected-writers 1' in prepare
    assert '--expected-writers 2' in probe


def test_arm_sdk_observer_reports_r1_weight_and_head_fields():
    """Diagnostics retain the exact R1 wrapper encoding and head slots."""
    for token in (
        'kHeadPitchSlot = 29',
        'kHeadYawSlot = 30',
        'command.mode_pr()',
        'static_cast<double>(result.mode_pr_min) / 100.0',
        'weight_encoding=r1_mode_pr_percent',
        'head_pitch_q_slot29',
        'head_yaw_q_slot30',
        'rate_hz=',
        'rt/arm/action/state',
        'map.find("holding")',
        'map.find("id")',
        'map.find("name")',
        'IDLE_FRESH_BOUNDED',
        'action_last_age_sec > options.action_fresh_sec',
    ):
        assert token in OBSERVER


def test_arm_sdk_observer_is_installed_and_preflight_fails_closed():
    """The bounded observer is a build target and a mandatory preflight gate."""
    assert 'add_executable(r1_arm_sdk_traffic_observer' in CMAKE
    assert 'R1_ARM_SDK_OBSERVER_BINARY=' in CMAKE
    assert OBSERVER_WRAPPER.is_file()
    wrapper = OBSERVER_WRAPPER.read_text()
    assert 'r1_arm_sdk_traffic_observer' in wrapper
    assert 'timeout --signal=TERM --kill-after=2s 40s "${OBSERVER_BINARY}"' in wrapper
    assert 'detail=wrapper_timeout' in wrapper
    preflight = PREFLIGHT.read_text()
    assert 'r1-arm-sdk-traffic-check' in preflight
    assert "10|11)" in preflight
    assert 'remote rt/arm_sdk command traffic is present' in preflight
    offline = OFFLINE_CHECK.read_text()
    assert 'r1_arm_sdk_traffic_observer' in offline
    assert 'observer RUNPATH' in offline


def test_physical_feedback_contract_has_26_joints():
    """The configured physical map retains the two head joints."""
    names = [
        line.strip() for line in CONFIG.splitlines()
        if line.strip().startswith('- "')
    ]
    assert len(names) == 36  # 26 physical + 10 arm names
    assert '      - "head_pitch_joint"' in CONFIG
    assert '      - "head_yaw_joint"' in CONFIG


def test_raw_firmware_modes_are_diagnostic_only():
    """LowState modes may be logged, but must never be treated as readiness."""
    assert 'mode_pr_raw=' in SOURCE
    assert 'mode_machine_raw=' in SOURCE
    assert 'must not reinterpret either value as "ready"' in SOURCE


def test_motor_health_is_derived_from_all_configured_lowstate_slots():
    """A dedicated Boolean must fail closed on stale data or any slot code."""
    for token in (
        '"motor_health_topic", "/r1/sdk_transport/motors_healthy"',
        'create_publisher<std_msgs::msg::Bool>',
        'motorstates[index] = motor.motorstate()',
        'motorstates[index] == 0U',
        'std::to_string(kPhysicalSlots[index]) + \':\'',
        'motorstate_nonzero=',
        'motorstate=unavailable',
        'publish_motor_health(feedback_ready && motors_healthy)',
    ):
        assert token in SOURCE
    assert 'motor_health_topic: "/r1/sdk_transport/motors_healthy"' in CONFIG
    assert "default_value='/r1/sdk_transport/motors_healthy'" in LAUNCH
    assert "'motor_health_topic': motor_health_topic" in LAUNCH


def test_preflight_requires_both_boolean_and_detailed_motor_health():
    """Physical preflight must independently verify health and diagnostics."""
    script = PREFLIGHT.read_text()
    for token in (
        'Fresh motorstate health',
        '/r1/sdk_transport/motors_healthy',
        'std_msgs/msg/Bool',
        "motor_health_dump == *'data: true'*",
        "status_dump == *'motorstate_nonzero=0'*",
        "status_dump == *'motorstate=none'*",
        'review reported slot:code details',
    ):
        assert token in script


def test_interlock_defaults_closed():
    """All paths that could lead to a writer remain disabled by default."""
    for key in (
        'sdk_enabled: false',
        'dry_run: true',
        'hardware_enabled: false',
        'commissioning_interlock: false',
        'arm_writer_enabled: false',
        'locomotion_writer_enabled: false',
    ):
        assert key in CONFIG


def test_preflight_is_read_only_and_fail_closed():
    """The commissioning helper must not contain a hardware writer path."""
    script = PREFLIGHT.read_text()
    assert PREFLIGHT.is_file()
    assert 'r1-sdk-readonly' in script
    assert 'rt/lf/lowstate' in script
    assert 'ArmSdk/LocoClient' in script
    assert 'r1_arm_sdk_dds_example' in script
    assert 'r1_loco_client' in script
    assert (
        'No actuator, locomotion, sport, or trajectory command was sent.'
        in script
    )


def test_runtime_abi_guard_precedes_every_runtime_boundary():
    """Loader/RMW checks must run before ROS and Unitree initialization."""
    main = SOURCE.index('int main(int argc, char ** argv)')
    preinit_guard = SOURCE.index(
        'r1_sdk_transport::verify_preinit_runtime_abi();', main)
    preinit_rmw_guard = SOURCE.index(
        'r1_sdk_transport::verify_active_rmw_implementation();',
        preinit_guard,
    )
    ros_init = SOURCE.index('rclcpp::init(argc, argv);', preinit_rmw_guard)
    runtime_try = SOURCE.rfind('try {', preinit_guard, ros_init)
    active_rmw_guard = SOURCE.index(
        'r1_sdk_transport::verify_active_rmw_implementation();', ros_init)
    node_construction = SOURCE.index(
        'std::make_shared<r1_sdk_transport::R1SdkTransport>()',
        active_rmw_guard,
    )
    assert preinit_guard < preinit_rmw_guard < runtime_try < ros_init
    assert ros_init < active_rmw_guard < node_construction
    assert 'r1_sdk_transport runtime failure:' in SOURCE[ros_init:]

    sdk_init = SOURCE.index('void initialize_sdk_read_only()')
    boundary_guard = SOURCE.index('verify_preinit_runtime_abi();', sdk_init)
    channel_factory = SOURCE.index(
        'ChannelFactory::Instance()->Init', boundary_guard)
    assert boundary_guard < channel_factory


def test_runtime_abi_guard_checks_rmw_and_canonical_loader_paths():
    """The guard pins FastRTPS and both members of the vendor DDS pair."""
    for token in (
        'RMW_IMPLEMENTATION',
        'rmw_fastrtps_cpp',
        'rmw_get_implementation_identifier()',
        'dladdr(',
        'dds_create_participant',
        'RTLD_NOLOAD',
        'RTLD_DI_LINKMAP',
        'std::filesystem::canonical',
        'R1_SDK_TRANSPORT_VENDOR_DDS_DIR',
    ):
        assert token in SOURCE


def test_build_and_launch_use_one_compiled_vendor_directory():
    """Build, guard, and launch must share the selected SDK path."""
    assert (
        'Legacy_Robotics/robotics/unitree_sdk/unitree_sdk2' in CMAKE
    )
    assert (
        'R1_SDK_TRANSPORT_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}"'
        in CMAKE
    )
    assert 'configure_file(' in CMAKE
    assert "VENDOR_DDS_DIRECTORY = '@UNITREE_SDK_LIB_DIR@'" in LAUNCH
    assert (
        "SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')"
        in LAUNCH
    )
    assert (
        "SetEnvironmentVariable('LD_LIBRARY_PATH', "
        "vendor_first_library_path())" in LAUNCH
    )
    package_xml = (PACKAGE / 'package.xml').read_text()
    assert '<exec_depend>rmw_fastrtps_cpp</exec_depend>' in package_xml
    assert '<exec_depend>ament_index_python</exec_depend>' in package_xml
