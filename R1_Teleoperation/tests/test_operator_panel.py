from operator_panel.commands import command_catalog, required_make_targets
from operator_panel.config import OperatorConfig


def test_catalog_contains_operator_actions_and_existing_targets():
    specs = {spec.key: spec for spec in command_catalog()}
    for key in (
        "check_all",
        "network",
        "robot_check",
        "stop",
        "kill",
        "arms_live",
        "legs_live",
        "teleop_live",
        "video_robot",
        "video_stereo",
    ):
        assert key in specs
    assert "robot-stop" in required_make_targets()
    assert "robot-kill" in required_make_targets()


def test_default_environment_is_fail_closed():
    env = OperatorConfig().as_environment()
    assert env["ROBOT_DRY_RUN"] == "1"
    assert env["ROBOT_ENABLE_ACTUATION"] == "0"
    assert env["ROBOT_CONFIRM_COMMISSIONING"] == "0"
    assert env["R1_ROBOT_IP"] == "192.168.123.164"
    assert env["R1_CONTROL_IP"] == "192.168.123.161"


def test_live_actions_are_marked_and_reset_kill_is_not_fabricated():
    specs = {spec.key: spec for spec in command_catalog()}
    assert specs["arms_live"].live
    assert specs["legs_live"].live
    assert specs["teleop_live"].live
    assert specs["reset_kill"].implemented is False
