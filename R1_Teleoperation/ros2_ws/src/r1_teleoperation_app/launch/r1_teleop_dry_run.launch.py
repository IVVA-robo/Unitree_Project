"""
Start the complete R1 VR teleoperation diagnostic pipeline in dry-run.

This launch deliberately composes only local ROS nodes and debug-only
outputs.  It never starts a Unitree ArmSdk or LocoClient, never publishes a
robot command topic, and forces the relevant environment interlocks to their
safe values.  The optional arm pipeline is read-only and disabled by default,
so this command works while the R1 is disconnected or charging.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package, relative_path):
    """Build a lazily resolved launch-file path for an installed package."""
    return PathJoinSubstitution(
        [FindPackageShare(package), *relative_path.split('/')]
    )


def _include(package, relative_path, launch_arguments=None, condition=None):
    """Create one included launch without resolving its package prematurely."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file(package, relative_path)),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    """Build the fail-closed, robot-writer-free teleoperation launch tree."""
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    udp_bind_address = LaunchConfiguration('udp_bind_address')
    udp_port = LaunchConfiguration('udp_port')
    discovery_port = LaunchConfiguration('discovery_port')
    vr_allowed_source_ip = LaunchConfiguration('vr_allowed_source_ip')
    start_bridge = LaunchConfiguration('start_bridge')
    start_head = LaunchConfiguration('start_head')
    start_locomotion = LaunchConfiguration('start_locomotion')
    start_arm_pipeline = LaunchConfiguration('start_arm_pipeline')
    locomotion_mode = LaunchConfiguration('locomotion_mode')
    arm_network_interface = LaunchConfiguration('arm_network_interface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='89',
            description=(
                'Local DDS domain for hardware-adjacent diagnostics; ROS is '
                'kept localhost-only.'
            ),
        ),
        DeclareLaunchArgument(
            'udp_bind_address',
            default_value='0.0.0.0',
            description='UDP bind address for the Pico/OpenXR bridge.',
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9090',
            description='UDP port for the Pico/OpenXR bridge.',
        ),
        DeclareLaunchArgument(
            'discovery_port',
            default_value='9091',
            description='UDP discovery port for the Pico/OpenXR bridge.',
        ),
        DeclareLaunchArgument(
            'vr_allowed_source_ip',
            default_value='',
            description=(
                'Optional fixed Pico IPv4 source. Empty is suitable only for '
                'dry-run/mock setup and locks the first valid sender.'
            ),
        ),
        DeclareLaunchArgument(
            'start_bridge',
            default_value='true',
            description='Start the UDP VR bridge, or use an already running one.',
        ),
        DeclareLaunchArgument(
            'start_head',
            default_value='true',
            description='Start the head-orientation debug controller.',
        ),
        DeclareLaunchArgument(
            'start_locomotion',
            default_value='true',
            description='Start the high-level locomotion debug controller.',
        ),
        DeclareLaunchArgument(
            'locomotion_mode',
            default_value='slow-safe',
            description='Debug profile: slow-safe, normal, or exhibition.',
        ),
        DeclareLaunchArgument(
            'start_arm_pipeline',
            default_value='false',
            description=(
                'Opt in to the read-only LowState plus arm-IK diagnostic '
                'pipeline. It needs a connected R1, but still has no writer.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_network_interface',
            default_value='enxb4b024be59fe',
            description=(
                'R1 Ethernet interface used only by the optional read-only '
                'LowState subscriber.'
            ),
        ),
        # The dry-run launch itself must remain dry-run even if a shell has
        # stale live values exported.  No included node receives a route to a
        # Unitree actuator writer.
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROBOT_DRY_RUN', '1'),
        SetEnvironmentVariable('ROBOT_ENABLE_ACTUATION', '0'),
        SetEnvironmentVariable('ROBOT_CONFIRM_OFF_CHARGER', '0'),
        SetEnvironmentVariable('ROBOT_CONFIRM_CLEAR_AREA', '0'),
        SetEnvironmentVariable('ROBOT_CONFIRM_ESTOP_READY', '0'),
        # The supervisor publishes a transient-local kill=true on startup.
        # The remaining nodes also fail closed when that message is absent.
        _include(
            'r1_teleop_safety',
            'launch/r1_safety_supervisor.launch.py',
        ),
        # A small wall-clock delay lets the supervisor publish its startup
        # kill latch before any diagnostic controller begins processing VR
        # input.  The controllers still have their own fail-closed defaults.
        TimerAction(
            period=0.25,
            actions=[
                _include(
                    'vr_teleop_bridge',
                    'launch/vr_bridge.launch.py',
                    {
                        'ros_localhost_only': '1',
                        'bind_address': udp_bind_address,
                        'udp_port': udp_port,
                        'discovery_port': discovery_port,
                        'allowed_source_ip': vr_allowed_source_ip,
                    },
                    IfCondition(start_bridge),
                ),
                _include(
                    'r1_hardware_adapter',
                    'launch/r1_head_dry_run.launch.py',
                    {'use_sim_time': 'false'},
                    IfCondition(start_head),
                ),
                _include(
                    'r1_hardware_adapter',
                    'launch/r1_locomotion_dry_run.launch.py',
                    {
                        'mode': locomotion_mode,
                        'input_topic': '/vr/cmd_vel',
                        'active_topic': '/vr/teleop/active',
                        'emergency_stop_topic': '/r1/safety/kill',
                    },
                    IfCondition(start_locomotion),
                ),
                _include(
                    'r1_hardware_adapter',
                    'launch/r1_hardware_dry_run.launch.py',
                    {'network_interface': arm_network_interface},
                    IfCondition(start_arm_pipeline),
                ),
            ],
        ),
    ])
