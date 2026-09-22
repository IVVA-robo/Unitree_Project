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
        'robot output is intentionally not connected',
    ):
        assert required in NODE
