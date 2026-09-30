import json

from operator_panel.config import OperatorConfig
from operator_panel.live_ack import (
    LIVE_ACK_MAX_AGE_SEC,
    acknowledgement_valid,
    clear_acknowledgement,
    record_acknowledgement,
)


def test_live_ack_is_bounded_and_bound_to_robot_identity(tmp_path):
    path = tmp_path / "ack.json"
    config = OperatorConfig(
        robot_ip="192.168.123.161",
        robot_interface="enxb4b024be59fe",
    )

    record_acknowledgement(config, path=path, now=100.0)

    assert acknowledgement_valid(config, path=path, now=101.0)
    assert not acknowledgement_valid(
        config, path=path, now=100.0 + LIVE_ACK_MAX_AGE_SEC + 0.1
    )
    other = OperatorConfig(
        robot_ip="192.168.123.162",
        robot_interface="enxb4b024be59fe",
    )
    assert not acknowledgement_valid(other, path=path, now=101.0)


def test_live_ack_rejects_future_corrupt_and_cleared_records(tmp_path):
    path = tmp_path / "ack.json"
    config = OperatorConfig()
    record_acknowledgement(config, path=path, now=200.0)
    assert not acknowledgement_valid(config, path=path, now=199.0)

    path.write_text(json.dumps({"schema": 1}), encoding="utf-8")
    assert not acknowledgement_valid(config, path=path, now=201.0)

    clear_acknowledgement(path=path)
    assert not path.exists()
