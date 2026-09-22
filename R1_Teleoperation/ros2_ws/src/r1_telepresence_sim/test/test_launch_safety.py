"""Static safety contracts for the direct Gazebo launch entry point."""

from pathlib import Path


def test_simulation_launch_forces_local_dds_and_namespaces_velocity():
    package = Path(__file__).parents[1]
    launch_source = (
        package / 'launch' / 'telepresence_sim.launch.py'
    ).read_text(encoding='utf-8')

    assert 'OpaqueFunction(function=_require_isolated_simulation)' in launch_source
    assert "SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1')" in launch_source
    assert "'cmd_vel_output_topic': '/r1/sim/safe_cmd_vel'" in launch_source
    assert "'output_topic': '/r1/sim/cmd_vel'" in launch_source
    assert "'arm_command_topic': '/r1/sim/ik_arm_trajectory'" in launch_source
    assert "'leg_input_topic': '/r1/telepresence/leg_targets'" in launch_source
    assert "'waist_hold_enabled': 'false'" in launch_source
    assert "executable='r1_sim_whole_body_planner'" in launch_source


def test_simulation_defaults_do_not_use_global_cmd_vel_topics():
    package = Path(__file__).parents[1]
    adapter_source = (
        package / 'r1_telepresence_sim' / 'locomotion_adapter.py'
    ).read_text(encoding='utf-8')
    visualizer_source = (
        package / 'r1_telepresence_sim' / 'leg_visualizer.py'
    ).read_text(encoding='utf-8')

    assert "'output_topic', '/r1/sim/cmd_vel'" in adapter_source
    assert "'input_topic', '/r1/sim/cmd_vel'" in visualizer_source
    planner_source = (
        package / 'r1_telepresence_sim' / 'whole_body_planner.py'
    ).read_text(encoding='utf-8')
    assert "'/arm_trajectory_controller/joint_trajectory'" in planner_source
    assert "'/waist_hold_controller/commands'" in planner_source
