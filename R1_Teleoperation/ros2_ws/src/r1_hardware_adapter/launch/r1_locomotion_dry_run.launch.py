"""Launch high-level locomotion diagnostics without a robot command path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start only the dry-run VR locomotion safety controller."""
    share = get_package_share_directory('r1_hardware_adapter')
    config = os.path.join(share, 'config', 'r1_locomotion_dry_run.yaml')
    return LaunchDescription([
        # VR ROS topics are local to the operator laptop.  No SDK, robot DDS,
        # or remote ROS endpoint is needed for this dry-run controller.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'mode',
            default_value='slow-safe',
            description='slow-safe, normal, or exhibition debug profile',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/vr/cmd_vel',
            description='VR TwistStamped input topic',
        ),
        DeclareLaunchArgument(
            'active_topic',
            default_value='/vr/locomotion/active',
            description='VR locomotion authorization Bool input topic',
        ),
        DeclareLaunchArgument(
            'emergency_stop_topic',
            default_value='/r1/safety/kill',
            description='Bool kill switch; true blocks all debug motion',
        ),
        Node(
            package='r1_hardware_adapter',
            executable='r1_locomotion_dry_run',
            name='r1_locomotion_dry_run',
            output='screen',
            parameters=[
                config,
                {
                    'hardware_enabled': False,
                    'dry_run': True,
                    'mode': LaunchConfiguration('mode'),
                    'input_topic': LaunchConfiguration('input_topic'),
                    'active_topic': LaunchConfiguration('active_topic'),
                    'emergency_stop_topic': LaunchConfiguration(
                        'emergency_stop_topic'
                    ),
                },
            ],
        ),
    ])
