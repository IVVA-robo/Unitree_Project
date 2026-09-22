"""Launch the read-only R1 LowState monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('r1_hardware_adapter')
    config = os.path.join(share, 'config', 'r1_state_monitor.yaml')
    return LaunchDescription([
        Node(
            package='r1_hardware_adapter',
            executable='r1_state_monitor',
            name='r1_state_monitor',
            output='screen',
            parameters=[config],
        ),
    ])

