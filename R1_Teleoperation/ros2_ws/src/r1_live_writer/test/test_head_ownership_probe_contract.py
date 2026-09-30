"""Focused offline contracts for the bounded ArmSdk ownership micro-probe."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src" / "r1_live_writer_node.cpp").read_text(encoding="utf-8")
CONFIG = (ROOT / "config" / "r1_live_writer.yaml").read_text(encoding="utf-8")
LAUNCH = (ROOT / "launch" / "r1_live_writer.launch.py").read_text(
    encoding="utf-8"
)


def function_body(source: str, signature: str) -> str:
    """Return one C++ function body for order-sensitive assertions."""
    signature_index = source.index(signature)
    opening_brace = source.index("{", signature_index)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError(f"unterminated C++ function: {signature}")


def test_legacy_recenter_rejects_and_only_new_service_starts_probe():
    legacy = function_body(NODE, "void on_recenter_head(")
    probe = function_body(NODE, "void on_probe_head_ownership(")
    assert "full physical recenter is suspended" in legacy
    assert "/r1/live_writer/probe_head_ownership" in legacy
    assert "HeadRecenterState::PendingFreshSeed" not in legacy
    assert "HeadRecenterState::PendingFreshSeed" in probe
    assert '"/r1/live_writer/recenter_head"' in NODE
    assert '"/r1/live_writer/probe_head_ownership"' in NODE


def test_physical_probe_requires_new_ack_and_probe_only_interlock():
    validation = function_body(NODE, "void validate_parameters() const")
    assert "head_ownership_probe_only_" in validation
    assert "head_ownership_probe_confirmed_" in validation
    assert "head_ownership_probe_environment_confirmed()" in validation
    assert "ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1" in validation
    assert "ROBOT_CONFIRM_HEAD_RECENTER=1" not in validation

    watchdog = function_body(NODE, "std::string active_watchdog_failure(")
    assert "head_ownership_probe_confirmed_" in watchdog
    assert "head_ownership_probe_environment_confirmed()" in watchdog
    assert "watchdog_head_ownership_probe_confirmation_lost" in watchdog

    for text in (CONFIG, LAUNCH):
        assert "head_ownership_probe_only" in text
        assert "head_ownership_probe_confirmed" in text


def test_probe_sequence_is_zero_weight_ramp_out_verify_return_confirm():
    probe = function_body(NODE, "void process_head_recenter(")
    ordered = (
        "seed_head_weighted(\n        head_recenter_seed_, 0.0)",
        "HeadRecenterState::ZeroWeightDiscovery",
        "HeadRecenterState::RampingWeight",
        "HeadRecenterState::ProbingOut",
        "HeadRecenterState::VerifyingOut",
        "HeadRecenterState::ReturningToSeed",
        "HeadRecenterState::ConfirmingReturn",
        "head_ownership_probe_verified awaiting_release=true",
        "head_ownership_probe=passed terminal=true",
    )
    positions = [probe.index(item) for item in ordered]
    assert positions == sorted(positions)
    assert "transport_->seed_head(" not in probe
    assert "transport_->command_head(" not in probe


def test_probe_accepts_fsm811_zero_without_calling_it_visual_center():
    service = function_body(NODE, "void on_probe_head_ownership(")
    probe = function_body(NODE, "void process_head_recenter(")
    assert "yaw is too close to zero" not in service
    assert "positive_room" in probe
    assert "negative_room" in probe
    assert "positive_room >= negative_room ? 1.0 : -1.0" in probe
    assert "insufficient_yaw_room_for_probe" in probe
    assert "head_absolute_limits_.yaw_max" in probe
    assert "head_absolute_limits_.yaw_min" in probe


def test_probe_immutable_ceilings_are_tiny_and_seed_is_stable_first():
    validation = function_body(NODE, "void validate_parameters() const")
    for ceiling in (
        "head_probe_zero_weight_hold_sec_ < 0.25",
        "head_probe_zero_weight_hold_sec_ > 0.50",
        "head_probe_step_rad_ > 0.005",
        "head_probe_rate_rad_s_ > 0.005",
        "head_probe_max_feedback_velocity_rad_s_ > 0.04",
        "head_probe_seed_stability_rad_ > 0.003",
        "head_probe_other_joint_tolerance_rad_ > 0.003",
        "head_probe_claim_tolerance_rad_ > 0.003",
        "head_probe_required_feedback_samples_ < 3",
        "head_probe_required_feedback_samples_ > 5",
        "head_probe_seed_samples_ < 10",
    ):
        assert ceiling in validation

    probe = function_body(NODE, "void process_head_recenter(")
    stable_window = probe.index("head_probe_seed_samples_")
    transport_boundary = probe.index("ensure_transport()", stable_window)
    weighted_seed = probe.index("seed_head_weighted(", transport_boundary)
    assert stable_window < transport_boundary < weighted_seed
    assert "full_seed_changed_before_zero_weight_boundary" in probe[
        transport_boundary:weighted_seed
    ]


def test_probe_uses_its_own_velocity_ceiling_and_outbound_baseline():
    feedback = function_body(NODE, "std::string head_probe_feedback_failure(")
    assert feedback.count("head_probe_max_feedback_velocity_rad_s_") >= 2
    assert "head_recenter_max_feedback_velocity_rad_s_" not in feedback
    assert "head_probe_max_feedback_velocity_rad_s: 0.04" in CONFIG

    probe = function_body(NODE, "void process_head_recenter(")
    ramp_complete = probe.index("if (weight >= 1.0)")
    baseline = probe.index(
        "head_probe_outbound_baseline_progress_rad_", ramp_complete
    )
    probe_out = probe.index("HeadRecenterState::ProbingOut", baseline)
    verify = probe.index("HeadRecenterState::VerifyingOut", probe_out)
    subtract = probe.index(
        "progress - head_probe_outbound_baseline_progress_rad_", verify
    )
    threshold = probe.index("head_probe_min_progress_rad_", subtract)
    assert ramp_complete < baseline < probe_out < verify < subtract < threshold

    excursion_guard = probe.index("yaw_exceeded_probe_excursion")
    guarded_prefix = probe[max(0, excursion_guard - 300):excursion_guard]
    assert "head_probe_step_magnitude_rad_" in guarded_prefix
    assert "head_probe_direction_tolerance_rad_" in guarded_prefix

    diagnostics = function_body(NODE, "std::string head_probe_diagnostics(")
    for field in (
        "outbound_progress_yaw=",
        "actual_excursion_yaw=",
        "actual_excursion_limit=",
        "feedback_yaw_velocity=",
        "feedback_pitch_velocity=",
        "feedback_velocity_limit=",
    ):
        assert field in diagnostics


def test_only_yaw_target_changes_and_other_twelve_fields_are_watched():
    publish = function_body(NODE, "bool publish_head_probe_frame(")
    measured_handoff = publish.index("held_head_target_ = latest_state_")
    settled_hold = publish.index("held_head_target_ = head_probe_full_weight_seed_")
    pitch_write = publish.index(
        "held_head_target_[kHeadPitchIndex]", settled_hold
    )
    yaw_write = publish.index("held_head_target_[kHeadYawIndex]", pitch_write)
    weighted_send = publish.index("command_head_weighted(", yaw_write)
    assert measured_handoff < settled_hold < pitch_write < yaw_write < weighted_send
    assert "single weight covers all 13 upper-body joints" in publish

    feedback = function_body(NODE, "std::string head_probe_feedback_failure(")
    assert "if (index == kHeadYawIndex)" in feedback
    assert "unexpected_other_joint_motion" in feedback
    assert "head_probe_other_joint_limit(index)" in feedback

    probe = function_body(NODE, "void process_head_recenter(")
    assert "head_probe_min_progress_rad_" in probe
    assert "outbound_feedback_reversed" in probe
    assert "yaw_moved_in_wrong_direction" in probe
    assert "yaw_exceeded_probe_excursion" in probe


def test_running_handoff_has_a_bounded_settling_envelope_then_restores_tight_guard():
    validation = function_body(NODE, "void validate_parameters()")
    assert "head_probe_ramp_other_joint_tolerance_rad_ > 0.02" in validation
    assert (
        "head_probe_ramp_other_joint_tolerance_rad_ <\n"
        "      head_probe_other_joint_tolerance_rad_"
    ) in validation

    limit = function_body(NODE, "double head_probe_other_joint_limit(")
    assert "HeadRecenterState::RampingWeight" in limit
    assert "index <= kWaistYawIndex" in limit
    assert "head_probe_ramp_other_joint_tolerance_rad_" in limit
    assert "head_probe_other_joint_tolerance_rad_" in limit

    probe = function_body(NODE, "void process_head_recenter(")
    ramp_complete = probe.index("if (weight >= 1.0)")
    capture = probe.index("head_probe_full_weight_seed_ = latest_state_", ramp_complete)
    valid = probe.index("head_probe_full_weight_seed_valid_ = true", capture)
    probe_out = probe.index("HeadRecenterState::ProbingOut", valid)
    assert ramp_complete < capture < valid < probe_out


def test_success_releases_then_reports_pass_and_latches_kill():
    probe = function_body(NODE, "void process_head_recenter(")
    verified = probe.index("head_ownership_probe_verified awaiting_release=true")
    stop = probe.rfind("safe_stop_outputs(", 0, verified)
    cleanup_check = probe.index("!outputs_active()", verified)
    passed = probe.index("head_ownership_probe=passed terminal=true", cleanup_check)
    failed = probe.index("cause=release_cleanup_pending", passed)
    assert stop < verified < cleanup_check < passed < failed
    assert probe.count('" cleanup={" + last_safe_stop_detail_ + "}"') == 2

    safe_stop = function_body(NODE, "void safe_stop_outputs(")
    hold = safe_stop.index("transport_->hold_head(")
    release = safe_stop.index("transport_->release_head(", hold)
    preserve_detail = safe_stop.index("last_safe_stop_detail_ = detail", release)
    kill = safe_stop.index("publish_kill_request(true)", preserve_detail)
    assert hold < release < preserve_detail < kill
    assert "transport_->seed_head(" not in safe_stop
    assert "transport_->command_head(" not in safe_stop
    assert "transport_->command_head_weighted(" not in safe_stop


def test_diagnostics_include_weight_target_feedback_and_other_joint_delta():
    diagnostics = function_body(NODE, "std::string head_probe_diagnostics(")
    for field in (
        "phase=",
        "weight=",
        "target_yaw=",
        "feedback_yaw=",
        "error_yaw=",
        "progress_yaw=",
        "max_other_joint=",
        "max_other_delta=",
        "feedback_samples=",
    ):
        assert field in diagnostics
