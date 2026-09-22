"""Launch only the dry-run VR head-orientation diagnostic node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build a head-only launch that cannot open an actuator writer."""
    share = get_package_share_directory('r1_hardware_adapter')
    config = os.path.join(share, 'config', 'r1_head_dry_run.yaml')
    return LaunchDescription([
        # Even a debug trajectory must not be discoverable by an accidental
        # ROS graph on the robot-facing Ethernet interface.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='r1_hardware_adapter',
            executable='r1_head_dry_run',
            name='r1_head_dry_run',
            output='screen',
            parameters=[
                config,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                    'hardware_enabled': False,
                    'dry_run': True,
                },
            ],
        ),
    ])
