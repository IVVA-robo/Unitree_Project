"""Launch the isolated R1 writer with all physical outputs disabled by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build a mock/dry-arm launch; callers must opt in to every live lock."""
    share = get_package_share_directory('r1_live_writer')
    config = os.path.join(share, 'config', 'r1_live_writer.yaml')

    arguments = [
        DeclareLaunchArgument('transport', default_value='mock'),
        DeclareLaunchArgument('response_config', default_value=config),
        DeclareLaunchArgument('arm_sdk_frame_profile', default_value='legacy'),
        DeclareLaunchArgument('sequential_control', default_value='false'),
        DeclareLaunchArgument('send_commands', default_value='false'),
        DeclareLaunchArgument('enable_head', default_value='false'),
        DeclareLaunchArgument('enable_arms', default_value='false'),
        DeclareLaunchArgument('enable_locomotion', default_value='false'),
        DeclareLaunchArgument(
            'locomotion_command_mode', default_value='loco_rpc'
        ),
        DeclareLaunchArgument(
            'wireless_controller_rate_hz', default_value='20.0'
        ),
        DeclareLaunchArgument(
            'velocity_status_127_probe_enabled', default_value='false'
        ),
        DeclareLaunchArgument('enable_prepare', default_value='false'),
        DeclareLaunchArgument(
            'prepare_enter_locomotion', default_value='false'
        ),
        DeclareLaunchArgument('prepare_speed_mode', default_value='-1'),
        DeclareLaunchArgument('static_prepare_mode', default_value='false'),
        DeclareLaunchArgument(
            'exhibition_session_mode', default_value='false'
        ),
        DeclareLaunchArgument(
            'exhibition_session_armed_on_start', default_value='false'
        ),
        DeclareLaunchArgument(
            'exhibition_reconnect_grace_sec', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'head_absolute_envelope_confirmed', default_value='false'
        ),
        DeclareLaunchArgument('head_recenter_enabled', default_value='false'),
        DeclareLaunchArgument('head_recenter_confirmed', default_value='false'),
        DeclareLaunchArgument(
            'head_ownership_probe_only', default_value='true'
        ),
        DeclareLaunchArgument(
            'head_ownership_probe_confirmed', default_value='false'
        ),
        DeclareLaunchArgument('commissioning_confirmed', default_value='false'),
        DeclareLaunchArgument('commissioning_token', default_value=''),
        DeclareLaunchArgument('expected_vr_source_ip', default_value=''),
        DeclareLaunchArgument('vr_transport', default_value='lan'),
        DeclareLaunchArgument('profile', default_value='slow-safe'),
        DeclareLaunchArgument(
            'arm_topic',
            default_value='/r1_kinematics_control/debug/arm_trajectory',
        ),
        DeclareLaunchArgument(
            'network_interface', default_value='enxb4b024be59fe'
        ),
        DeclareLaunchArgument(
            'motor_health_topic',
            default_value='/r1/sdk_transport/motors_healthy',
        ),
        DeclareLaunchArgument('max_forward_mps', default_value='0.20'),
        DeclareLaunchArgument('max_lateral_mps', default_value='0.12'),
        DeclareLaunchArgument('max_yaw_rps', default_value='0.35'),
    ]
    boolean = lambda name: ParameterValue(
        LaunchConfiguration(name), value_type=bool
    )
    node = Node(
        package='r1_live_writer',
        executable='r1_live_writer_node',
        name='r1_live_writer',
        output='screen',
        parameters=[
            config,
            LaunchConfiguration('response_config'),
            {
                'transport': LaunchConfiguration('transport'),
                'arm_sdk_frame_profile': LaunchConfiguration('arm_sdk_frame_profile'),
                'sequential_control': LaunchConfiguration('sequential_control'),
                'send_commands': boolean('send_commands'),
                'enable_head': boolean('enable_head'),
                'enable_arms': boolean('enable_arms'),
                'enable_locomotion': boolean('enable_locomotion'),
                'locomotion_command_mode': LaunchConfiguration(
                    'locomotion_command_mode'
                ),
                'wireless_controller_rate_hz': ParameterValue(
                    LaunchConfiguration('wireless_controller_rate_hz'),
                    value_type=float,
                ),
                'velocity_status_127_probe_enabled': boolean(
                    'velocity_status_127_probe_enabled'
                ),
                'enable_prepare': boolean('enable_prepare'),
                'prepare_enter_locomotion': boolean(
                    'prepare_enter_locomotion'
                ),
                'prepare_speed_mode': ParameterValue(
                    LaunchConfiguration('prepare_speed_mode'), value_type=int
                ),
                'static_prepare_mode': boolean('static_prepare_mode'),
                'exhibition_session_mode': boolean(
                    'exhibition_session_mode'
                ),
                'exhibition_session_armed_on_start': boolean(
                    'exhibition_session_armed_on_start'
                ),
                'exhibition_reconnect_grace_sec': ParameterValue(
                    LaunchConfiguration('exhibition_reconnect_grace_sec'),
                    value_type=float,
                ),
                'head_absolute_envelope_confirmed': boolean(
                    'head_absolute_envelope_confirmed'
                ),
                'head_recenter_enabled': boolean('head_recenter_enabled'),
                'head_recenter_confirmed': boolean(
                    'head_recenter_confirmed'
                ),
                'head_ownership_probe_only': boolean(
                    'head_ownership_probe_only'
                ),
                'head_ownership_probe_confirmed': boolean(
                    'head_ownership_probe_confirmed'
                ),
                'commissioning_confirmed': boolean(
                    'commissioning_confirmed'
                ),
                'commissioning_token': LaunchConfiguration(
                    'commissioning_token'
                ),
                'expected_vr_source_ip': LaunchConfiguration(
                    'expected_vr_source_ip'
                ),
                'profile': LaunchConfiguration('profile'),
                'vr_transport': LaunchConfiguration('vr_transport'),
                'arm_topic': LaunchConfiguration('arm_topic'),
                'network_interface': LaunchConfiguration('network_interface'),
                'motor_health_topic': LaunchConfiguration(
                    'motor_health_topic'
                ),
                'max_forward_mps': ParameterValue(
                    LaunchConfiguration('max_forward_mps'), value_type=float
                ),
                'max_lateral_mps': ParameterValue(
                    LaunchConfiguration('max_lateral_mps'), value_type=float
                ),
                'max_yaw_rps': ParameterValue(
                    LaunchConfiguration('max_yaw_rps'), value_type=float
                ),
            },
        ],
    )
    return LaunchDescription([*arguments, node])
