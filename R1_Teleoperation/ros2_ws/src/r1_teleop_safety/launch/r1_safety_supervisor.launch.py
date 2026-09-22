"""Launch the fail-closed R1 teleoperation safety supervisor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Create only the local safety-kill latch; no hardware client exists."""
    config = os.path.join(
        get_package_share_directory('r1_teleop_safety'),
        'config',
        'r1_teleop_safety.yaml',
    )
    return LaunchDescription([
        # The software interlock belongs to the operator laptop's local ROS
        # graph; it must not accidentally participate in robot DDS discovery.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        Node(
            package='r1_teleop_safety',
            executable='r1_safety_supervisor',
            name='r1_safety_supervisor',
            output='screen',
            parameters=[config],
        ),
    ])
