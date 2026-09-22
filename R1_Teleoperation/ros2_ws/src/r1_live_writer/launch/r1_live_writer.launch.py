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
        DeclareLaunchArgument('send_commands', default_value='false'),
        DeclareLaunchArgument('enable_head', default_value='false'),
        DeclareLaunchArgument('enable_arms', default_value='false'),
        DeclareLaunchArgument('enable_locomotion', default_value='false'),
        DeclareLaunchArgument('enable_prepare', default_value='false'),
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
            {
                'transport': LaunchConfiguration('transport'),
                'send_commands': boolean('send_commands'),
                'enable_head': boolean('enable_head'),
                'enable_arms': boolean('enable_arms'),
                'enable_locomotion': boolean('enable_locomotion'),
                'enable_prepare': boolean('enable_prepare'),
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
                'arm_topic': LaunchConfiguration('arm_topic'),
                'network_interface': LaunchConfiguration('network_interface'),
                'motor_health_topic': LaunchConfiguration(
                    'motor_health_topic'
                ),
            },
        ],
    )
    return LaunchDescription([*arguments, node])
