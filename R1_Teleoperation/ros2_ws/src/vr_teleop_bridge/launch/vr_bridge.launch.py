from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
            'discovery_port',
            default_value='9091',
            description='UDP discovery port advertised to the Pico sender',
        ),
        DeclareLaunchArgument(
            'allowed_source_ip',
            default_value='',
            description=(
                'Optional fixed Pico IPv4 address. An empty value accepts the '
                'first valid sender and locks it for that bridge process.'
            ),
        ),
        DeclareLaunchArgument(
            'activation_mode',
            default_value=EnvironmentVariable(
                'R1_EXHIBITION_SESSION_MODE', default_value='deadman'
            ),
            description='deadman (default) or explicit exhibition session_arm',
        ),
        DeclareLaunchArgument('pause_on_packet_timeout', default_value='false'),
        DeclareLaunchArgument(
            'tracking_grace_sec',
            default_value=EnvironmentVariable(
                'R1_EXHIBITION_TRACKING_GRACE_SEC', default_value='2.0'
            ),
            description='Operator status grace before reconnecting becomes holding',
        ),
        DeclareLaunchArgument(
            'recovery_blend_sec',
            default_value=EnvironmentVariable(
                'R1_EXHIBITION_RECOVERY_BLEND_SEC', default_value='0.5'
            ),
            description='Bounded pose blend after XR tracking returns',
        ),
        DeclareLaunchArgument(
            'safety_profile',
            default_value=EnvironmentVariable(
                'SAFETY_PROFILE', default_value='standard'
            ),
            description='standard or the reconnect-tolerant exhibition profile',
        ),
        DeclareLaunchArgument(
            'max_forward_mps',
            default_value='0.35',
            description='Maximum forward/backward VR velocity intent',
        ),
        DeclareLaunchArgument(
            'max_lateral_mps',
            default_value='0.25',
            description='Maximum lateral VR velocity intent',
        ),
        DeclareLaunchArgument(
            'max_yaw_rps',
            default_value='0.60',
            description='Maximum VR yaw-rate intent',
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
                    'discovery_port': ParameterValue(
                        LaunchConfiguration('discovery_port'), value_type=int
                    ),
                    'allowed_source_ip': LaunchConfiguration('allowed_source_ip'),
                    'activation_mode': LaunchConfiguration('activation_mode'),
                    'pause_on_packet_timeout': ParameterValue(
                        LaunchConfiguration('pause_on_packet_timeout'), value_type=bool
                    ),
                    'tracking_grace_sec': ParameterValue(
                        LaunchConfiguration('tracking_grace_sec'), value_type=float
                    ),
                    'recovery_blend_sec': ParameterValue(
                        LaunchConfiguration('recovery_blend_sec'), value_type=float
                    ),
                    'safety_profile': LaunchConfiguration('safety_profile'),
                    'max_forward_mps': ParameterValue(
                        LaunchConfiguration('max_forward_mps'), value_type=float
                    ),
                    'max_lateral_mps': ParameterValue(
                        LaunchConfiguration('max_lateral_mps'), value_type=float
                    ),
                    'max_yaw_rps': ParameterValue(
                        LaunchConfiguration('max_yaw_rps'), value_type=float
                    ),
                },
            ],
        ),
    ])
