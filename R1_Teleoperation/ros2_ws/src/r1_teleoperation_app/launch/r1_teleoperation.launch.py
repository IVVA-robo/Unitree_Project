"""
Start the complete R1 teleoperation demo from one safe launch command.

The launch composes the already-tested VR bridge, Gazebo simulation, headset-
relative IK/leg adapters, and the video-only POV server.  It intentionally does
not include the hardware adapter or any Unitree motor/DDS client.  Thus the
default command is suitable for a laptop with a real robot nearby: all ROS
traffic is localhost-only and all actuator output terminates in Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package, relative_path):
    """Resolve one launch file from an installed ROS package."""
    return os.path.join(
        get_package_share_directory(package),
        *relative_path.split('/'),
    )


def generate_launch_description():
    """Build the one-command, simulation-only teleoperation launch tree."""
    ros_localhost_only = LaunchConfiguration('ros_localhost_only')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    start_bridge = LaunchConfiguration('start_bridge')
    start_simulation = LaunchConfiguration('start_simulation')
    start_video = LaunchConfiguration('start_video')

    video_env_default = os.path.join(
        get_package_share_directory('r1_robot_pov'),
        'config',
        'robot_pov.sim.env',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='81',
            description='DDS domain reserved for the local teleoperation demo',
        ),
        DeclareLaunchArgument(
            'ros_localhost_only',
            default_value='1',
            description=(
                'Keep ROS 2 discovery and commands on this laptop. UDP from '
                'the Pico is unaffected.'
            ),
        ),
        DeclareLaunchArgument(
            'gazebo_port',
            default_value='11347',
            description='Local Gazebo master port for this application',
        ),
        DeclareLaunchArgument(
            'udp_bind_address',
            default_value='0.0.0.0',
            description='UDP bind address for the Pico bridge',
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9090',
            description='UDP port for the Pico bridge',
        ),
        DeclareLaunchArgument(
            'start_bridge',
            default_value='true',
            description='Start the Pico UDP to ROS 2 bridge',
        ),
        DeclareLaunchArgument(
            'start_simulation',
            default_value='true',
            description='Start Gazebo, R1 controllers, IK and leg visualizer',
        ),
        DeclareLaunchArgument(
            'start_video',
            default_value='true',
            description='Start the video-only POV web server',
        ),
        DeclareLaunchArgument(
            'hardware_enabled',
            default_value='false',
            description=(
                'Must stay false. The simulation adapter rejects true and this '
                'launch never starts a hardware transport.'
            ),
        ),
        DeclareLaunchArgument(
            'display',
            default_value='false',
            description='Show the optional desktop OpenCV stereo window',
        ),
        DeclareLaunchArgument(
            'model_z',
            default_value='0.25',
            description='Raise the Gazebo model so leg motion is visible',
        ),
        DeclareLaunchArgument(
            'headset_relative_enabled',
            default_value='true',
            description='Use HMD-relative hand/body coordinates',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='~/.ros/r1_telepresence_sim_body_calibration.json',
            description='Simulation-only body calibration file',
        ),
        DeclareLaunchArgument(
            'video_env_file',
            default_value=video_env_default,
            description=(
                'POV dotenv file. The default subscribes to Gazebo ROS camera '
                'topics; use an explicit profile for another source.'
            ),
        ),
        DeclareLaunchArgument(
            'video_source',
            default_value='ros',
            description='POV source override: mock, ros, usb, rtsp or unitree',
        ),
        DeclareLaunchArgument('video_profile', default_value='low-latency'),
        DeclareLaunchArgument('video_transport', default_value='auto'),
        DeclareLaunchArgument('video_layout', default_value='stereo'),
        DeclareLaunchArgument('video_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument(
            'video_mdns',
            default_value='false',
            description='Advertise robot-pov.local through Avahi',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', ros_localhost_only),
        SetEnvironmentVariable(
            'GAZEBO_MASTER_URI',
            ['http://127.0.0.1:', LaunchConfiguration('gazebo_port')],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('vr_teleop_bridge', 'launch/vr_bridge.launch.py')
            ),
            launch_arguments={
                'ros_localhost_only': ros_localhost_only,
                'bind_address': LaunchConfiguration('udp_bind_address'),
                'udp_port': LaunchConfiguration('udp_port'),
            }.items(),
            condition=IfCondition(start_bridge),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file(
                    'r1_telepresence_sim',
                    'launch/telepresence_sim.launch.py',
                )
            ),
            launch_arguments={
                'ros_localhost_only': ros_localhost_only,
                'hardware_enabled': LaunchConfiguration('hardware_enabled'),
                'display': LaunchConfiguration('display'),
                'model_z': LaunchConfiguration('model_z'),
                'headset_relative_enabled': LaunchConfiguration(
                    'headset_relative_enabled'
                ),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }.items(),
            condition=IfCondition(start_simulation),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('r1_robot_pov', 'launch/robot_pov.launch.py')
            ),
            launch_arguments={
                'env_file': LaunchConfiguration('video_env_file'),
                'source': LaunchConfiguration('video_source'),
                'profile': LaunchConfiguration('video_profile'),
                'transport': LaunchConfiguration('video_transport'),
                'layout': LaunchConfiguration('video_layout'),
                'host': LaunchConfiguration('video_host'),
                'port': LaunchConfiguration('video_port'),
                'enable_mdns': LaunchConfiguration('video_mdns'),
            }.items(),
            condition=IfCondition(start_video),
        ),
    ])
