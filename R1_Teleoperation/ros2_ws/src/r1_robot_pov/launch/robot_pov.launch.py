"""Strictly video-only launch; no teleop, IK, or actuator node is started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_override(arguments, context, option, launch_name):
    value = LaunchConfiguration(launch_name).perform(context).strip()
    if value:
        arguments.extend((option, value))


def _video_only_node(context):
    arguments = [
        'serve',
        '--env-file',
        LaunchConfiguration('env_file').perform(context),
    ]
    for option, launch_name in (
        ('--source', 'source'),
        ('--profile', 'profile'),
        ('--transport', 'transport'),
        ('--layout', 'layout'),
        ('--host', 'host'),
        ('--port', 'port'),
    ):
        _optional_override(arguments, context, option, launch_name)

    enable_mdns = LaunchConfiguration('enable_mdns').perform(context).lower()
    if enable_mdns not in ('true', 'false'):
        raise ValueError('enable_mdns must be true or false')
    if enable_mdns == 'false':
        arguments.append('--no-mdns')

    enable_discovery = LaunchConfiguration('enable_discovery').perform(context).lower()
    if enable_discovery not in ('true', 'false'):
        raise ValueError('enable_discovery must be true or false')
    if enable_discovery == 'false':
        arguments.append('--no-discovery')

    return [Node(
        package='r1_robot_pov',
        executable='r1_robot_pov',
        name='r1_robot_pov_video_only',
        arguments=arguments,
        output='screen',
        # Robot POV is read-only and independent from the healthy VR bridge.
        # Let launch recover this child alone instead of cycling the complete
        # offline service (and therefore UDP/VR) after a camera-process crash.
        respawn=True,
        respawn_delay=2.0,
    )]


def generate_launch_description():
    default_env = PathJoinSubstitution([
        FindPackageShare('r1_robot_pov'),
        'config',
        'robot_pov.exhibition.env',
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'env_file',
            default_value=default_env,
            description='Absolute path to the Robot POV dotenv configuration',
        ),
        DeclareLaunchArgument(
            'source', default_value='',
            description='Optional mock/ros/usb/rtsp/unitree override',
        ),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='Optional quality-profile override',
        ),
        DeclareLaunchArgument(
            'transport', default_value='',
            description='Optional auto/webrtc/mjpeg override',
        ),
        DeclareLaunchArgument(
            'layout', default_value='',
            description='Optional mono/stereo override',
        ),
        DeclareLaunchArgument(
            'host', default_value='',
            description='Optional HTTP bind-address override',
        ),
        DeclareLaunchArgument(
            'port', default_value='',
            description='Optional HTTP port override',
        ),
        DeclareLaunchArgument(
            'enable_mdns', default_value='true',
            description='Publish robot-pov.local through Avahi when available',
        ),
        DeclareLaunchArgument(
            'enable_discovery', default_value='true',
            description='Bind UDP 9091 discovery responder',
        ),
        OpaqueFunction(function=_video_only_node),
    ])
