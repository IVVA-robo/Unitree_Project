from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    share = get_package_share_directory('vr_teleop_bridge')
    config = os.path.join(share, 'config', 'vr_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_localhost_only',
            default_value='1',
            description=(
                'Keep ROS 2/DDS outputs on this computer. The Pico UDP '
                'listener remains reachable over Wi-Fi.'
            ),
        ),
        DeclareLaunchArgument(
            'bind_address',
            default_value='0.0.0.0',
            description='UDP bind address for the Pico sender',
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='9090',
            description='UDP port for the Pico sender',
        ),
        DeclareLaunchArgument(
            'allowed_source_ip',
            default_value='',
            description=(
                'Optional fixed Pico IPv4 address. An empty value accepts the '
                'first valid sender and locks it for that bridge process.'
            ),
        ),
        SetEnvironmentVariable(
            name='ROS_LOCALHOST_ONLY',
            value=LaunchConfiguration('ros_localhost_only'),
        ),
        Node(
            package='vr_teleop_bridge',
            executable='vr_bridge',
            name='vr_teleop_bridge',
            output='screen',
            parameters=[
                config,
                {
                    'bind_address': LaunchConfiguration('bind_address'),
                    'udp_port': ParameterValue(
                        LaunchConfiguration('udp_port'), value_type=int
                    ),
                    'allowed_source_ip': LaunchConfiguration('allowed_source_ip'),
                },
            ],
        ),
    ])
