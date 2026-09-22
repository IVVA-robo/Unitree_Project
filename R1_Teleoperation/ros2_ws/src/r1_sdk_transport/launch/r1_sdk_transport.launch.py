"""Launch the R1 SDK transport in read-only commissioning mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VENDOR_DDS_DIRECTORY = '@UNITREE_SDK_LIB_DIR@'


def vendor_first_library_path():
    """Return a deterministic loader path with the compiled DDS pair first."""
    inherited = os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    remainder = [
        entry for entry in inherited
        if entry and entry != VENDOR_DDS_DIRECTORY
    ]
    return os.pathsep.join([VENDOR_DDS_DIRECTORY, *remainder])


def generate_launch_description():
    """Build an isolated read-only LowState and ROS diagnostics process."""
    share = get_package_share_directory('r1_sdk_transport')
    config = os.path.join(share, 'config', 'r1_sdk_transport.yaml')
    interface = LaunchConfiguration('network_interface')
    sdk_enabled = LaunchConfiguration('sdk_enabled')
    motor_health_topic = LaunchConfiguration('motor_health_topic')
    return LaunchDescription([
        # The executable independently verifies both settings and fails before
        # rclcpp/ChannelFactory initialization if either is ineffective.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', vendor_first_library_path()),
        # This only isolates ROS 2 graph traffic.  The vendor SDK still uses
        # the explicit Ethernet interface and creates a reader, never a writer.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'network_interface',
            default_value='enxb4b024be59fe',
            description='Ethernet interface connected to the physical R1',
        ),
        DeclareLaunchArgument(
            'sdk_enabled',
            default_value='false',
            description='Enable read-only rt/lf/lowstate subscription',
        ),
        DeclareLaunchArgument(
            'motor_health_topic',
            default_value='/r1/sdk_transport/motors_healthy',
            description='Fail-closed health derived from all motorstate fields',
        ),
        Node(
            package='r1_sdk_transport',
            executable='r1_sdk_transport',
            name='r1_sdk_transport',
            output='screen',
            parameters=[
                config,
                {
                    'sdk_enabled': ParameterValue(
                        sdk_enabled, value_type=bool),
                    'network_interface': interface,
                    'motor_health_topic': motor_health_topic,
                    # These are hard-coded safety values in this launch; a
                    # writer cannot be enabled through launch arguments.
                    'dry_run': True,
                    'hardware_enabled': False,
                    'commissioning_interlock': False,
                    'arm_writer_enabled': False,
                    'locomotion_writer_enabled': False,
                },
            ],
        ),
    ])
