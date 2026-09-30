"""Static diagnostics contract for the UDP bridge."""

from pathlib import Path


NODE = (
    Path(__file__).resolve().parents[1]
    / 'vr_teleop_bridge'
    / 'node.py'
).read_text(encoding='utf-8')


def test_bridge_exposes_packet_health_without_enabling_robot_output():
    """Operators need source, freshness, and counters before commissioning."""
    for required in (
        "'/vr/teleop/status'",
        "'R1_TELEOP_DISCOVER'",
        "'R1_TELEOP_ENDPOINT'",
        "discovery_port",
        'source=',
        'valid_packets=',
        'bad_packets=',
        'last_sequence=',
        'packet_age_sec=',
        'fresh=',
        'packet_deadman=',
        'left_stick=',
        'right_stick=',
        'stick_input_scale=',
        "declare_parameter('stick_input_scale', 4.0)",
        'VR locomotion stick motion:',
        'VR locomotion sticks returned to neutral',
        'left_tracked=',
        'right_tracked=',
        'head_tracked=',
        'robot output is intentionally not connected',
    ):
        assert required in NODE


def test_exhibition_session_is_opt_in_and_separates_locomotion_authority():
    """Session hold cannot alter the default Deadman commissioning path."""
    for required in (
        "declare_parameter('activation_mode', 'deadman')",
        "self._activation_mode == 'session_arm'",
        "'/vr/teleop/arm_session'",
        "'/vr/teleop/disarm_session'",
        "'/vr/teleop/pause_session'",
        "'/vr/teleop/resume_session'",
        "'/vr/teleop/session_armed'",
        "'/vr/locomotion/active'",
        'DurabilityPolicy.TRANSIENT_LOCAL',
        'locomotion forced to zero',
        'tracking_flags_missing_update_vr_app',
        'locomotion_controls_not_neutral',
    ):
        assert required in NODE or required in (
            Path(__file__).resolve().parents[1]
            / 'vr_teleop_bridge'
            / 'resilience.py'
        ).read_text(encoding='utf-8')


def test_x_and_b_actions_have_separate_neutral_and_emergency_paths():
    for required in (
        "'/vr/actions/arms_neutral'",
        "'/vr/actions/emergency_stop'",
        "'/vr/actions/status'",
        "'/r1/safety/kill_request'",
        "'/vr/teleop/clear_emergency_stop'",
        'arms_reset_to_neutral',
        'emergency_stop_right_b',
        'self._velocity_message(stamp, None)',
        'self._session_gate.disarm()',
        "declare_parameter('safety_profile', 'standard')",
    ):
        assert required in NODE
