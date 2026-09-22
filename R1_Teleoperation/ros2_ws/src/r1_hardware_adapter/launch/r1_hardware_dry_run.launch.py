"""Launch the physical-R1 telemetry and VR pipeline without command writers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build a fail-safe, diagnostics-only pipeline for a connected R1."""
    adapter_share = get_package_share_directory('r1_hardware_adapter')
    kinematics_share = get_package_share_directory('r1_kinematics_control')
    description_share = get_package_share_directory('unitree_r1_description')

    state_config = os.path.join(
        adapter_share, 'config', 'r1_state_monitor.yaml'
    )
    adapter_config = os.path.join(
        adapter_share, 'config', 'r1_hardware_adapter.yaml'
    )
    kinematics_config = os.path.join(
        kinematics_share, 'config', 'r1_control.yaml'
    )
    default_urdf = os.path.join(description_share, 'urdf', 'r1.urdf')

    network_interface = LaunchConfiguration('network_interface')
    urdf_path = LaunchConfiguration('urdf_path')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        # ROS diagnostics stay on this laptop. The SDK subscriber below still
        # uses its explicitly configured Ethernet interface for R1 LowState.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'network_interface',
            default_value='enxb4b024be59fe',
            description='Ethernet interface connected to the physical R1',
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='Expanded R1 URDF used for diagnostics-only IK',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '~/Unitree_Project/Configs/Calibration/'
                'r1_body_calibration.json'
            ),
        ),
        Node(
            package='r1_hardware_adapter',
            executable='r1_state_monitor',
            name='r1_state_monitor',
            output='screen',
            parameters=[
                state_config,
                {'network_interface': network_interface},
            ],
        ),
        Node(
            package='r1_kinematics_control',
            executable='r1_kinematics_control',
            name='r1_kinematics_control',
            output='screen',
            parameters=[
                kinematics_config,
                {
                    'use_sim_time': False,
                    'dry_run': True,
                    'urdf_path': urdf_path,
                    'joint_states_topic': '/r1/hardware/joint_states',
                    'waist_hold_enabled': False,
                    'body_proxy.calibration_file': calibration_file,
                },
            ],
        ),
        Node(
            package='r1_hardware_adapter',
            executable='r1_hardware_adapter',
            name='r1_hardware_adapter',
            output='screen',
            parameters=[
                adapter_config,
                {
                    'use_sim_time': False,
                    'hardware_enabled': False,
                    'dry_run': True,
                    'arm_input_topic': (
                        '/r1_kinematics_control/debug/arm_trajectory'
                    ),
                    'velocity_input_topic': (
                        '/r1_kinematics_control/debug/cmd_vel'
                    ),
                },
            ],
        ),
    ])
