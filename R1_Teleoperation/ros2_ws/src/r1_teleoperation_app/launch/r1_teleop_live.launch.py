"""Compose the staged physical-R1 pipeline with every writer lock closed."""

import os
import platform
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_REQUIRED_RMW = 'rmw_fastrtps_cpp'
_FAST_DDS_RMW_LIBRARY = Path(
    '/opt/ros/humble/lib/librmw_fastrtps_cpp.so'
)
_DEFAULT_UNITREE_SDK_ROOT = Path(
    '/home/unitree/Unitree_Project/Legacy_Robotics/robotics/'
    'unitree_sdk/unitree_sdk2'
)
_REQUIRED_VENDOR_LIBRARIES = ('libddsc.so.0', 'libddscxx.so.0')


def _sdk_architecture(machine=None):
    """Map supported kernel architecture names to Unitree SDK directories."""
    machine = machine or platform.machine()
    aliases = {
        'x86_64': 'x86_64',
        'amd64': 'x86_64',
        'aarch64': 'aarch64',
        'arm64': 'aarch64',
    }
    try:
        return aliases[machine.lower()]
    except KeyError as exception:
        raise RuntimeError(
            '[BLOCKED] unsupported Unitree SDK architecture: '
            f'{machine}. No Unitree process was started.'
        ) from exception


def _canonical_directory(value, name):
    """Resolve one required absolute directory or fail before process start."""
    path = Path(value)
    if not path.is_absolute():
        raise RuntimeError(
            f'[BLOCKED] {name} must be an absolute path: {value}. '
            'No Unitree process was started.'
        )
    try:
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError) as exception:
        raise RuntimeError(
            f'[BLOCKED] {name} does not exist: {value}. '
            'No Unitree process was started.'
        ) from exception
    if not resolved.is_dir():
        raise RuntimeError(
            f'[BLOCKED] {name} is not a directory: {resolved}. '
            'No Unitree process was started.'
        )
    return resolved


def _prepend_library_directory(directory, current_value):
    """Put one canonical directory first and remove its exact duplicates."""
    directory_text = str(directory)
    remaining = [
        entry for entry in current_value.split(os.pathsep)
        if entry and entry != directory_text
    ]
    return os.pathsep.join([directory_text, *remaining])


def _physical_runtime_environment(environment, machine=None):
    """Return a complete, matched ROS/Unitree loader environment."""
    if not _FAST_DDS_RMW_LIBRARY.is_file():
        raise RuntimeError(
            '[BLOCKED] ROS Fast DDS RMW is unavailable: '
            f'{_FAST_DDS_RMW_LIBRARY}. Install '
            'ros-humble-rmw-fastrtps-cpp. No Unitree process was started.'
        )

    sdk_root_value = environment.get(
        'R1_UNITREE_SDK_ROOT', str(_DEFAULT_UNITREE_SDK_ROOT)
    )
    sdk_root = _canonical_directory(
        sdk_root_value, 'R1_UNITREE_SDK_ROOT'
    )
    vendor_directory = _canonical_directory(
        sdk_root / 'thirdparty' / 'lib' / _sdk_architecture(machine),
        'Unitree DDS library directory',
    )
    for library_name in _REQUIRED_VENDOR_LIBRARIES:
        library = vendor_directory / library_name
        if not library.is_file():
            raise RuntimeError(
                '[BLOCKED] Unitree DDS runtime is incomplete: '
                f'{library}. No Unitree process was started.'
            )

    return {
        # ROS must not load its CycloneDDS into a process that also links the
        # older Unitree CycloneDDS C++ library.
        'RMW_IMPLEMENTATION': _REQUIRED_RMW,
        'R1_UNITREE_SDK_ROOT': str(sdk_root),
        'R1_UNITREE_DDS_LIB_DIR': str(vendor_directory),
        'LD_LIBRARY_PATH': _prepend_library_directory(
            vendor_directory, environment.get('LD_LIBRARY_PATH', '')
        ),
        # This is consumed by r1_live_writer before rclcpp::init.  It is an
        # ABI safety marker, not authorization for a physical command.
        'R1_PHYSICAL_SDK_SESSION': '1',
    }


def _configure_physical_runtime(context):
    """Validate and pin the loader environment before any include can run."""
    environment = _physical_runtime_environment(context.environment)
    return [
        SetEnvironmentVariable(name, value)
        for name, value in environment.items()
    ]


def _launch_file(package, relative_path):
    """Resolve an installed launch file lazily."""
    return PathJoinSubstitution(
        [FindPackageShare(package), *relative_path.split('/')]
    )


def _include(package, relative_path, arguments=None, condition=None):
    """Include one launch description without resolving it during import."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file(package, relative_path)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    """Build a live-capable graph whose defaults cannot construct a writer."""
    domain = LaunchConfiguration('ros_domain_id')
    interface = LaunchConfiguration('network_interface')
    source_ip = LaunchConfiguration('vr_source_ip')
    start_readonly_reader = LaunchConfiguration('start_readonly_reader')
    motor_health_topic = LaunchConfiguration('motor_health_topic')
    arm_trajectory_topic = LaunchConfiguration('arm_topic')
    # The R1 description is installed in the already-sourced Legacy Robotics
    # workspace on this commissioning laptop. Keep the path explicit here so
    # a live launch can still start its fail-closed graph when that package is
    # not in the overlay's ament index; callers may override it.
    r1_urdf = LaunchConfiguration('urdf_path')

    declarations = [
        DeclareLaunchArgument('ros_domain_id', default_value='88'),
        DeclareLaunchArgument('network_interface',
                              default_value='enxb4b024be59fe'),
        DeclareLaunchArgument('udp_bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_port', default_value='9090'),
        DeclareLaunchArgument('vr_source_ip', default_value=''),
        DeclareLaunchArgument(
            'motor_health_topic',
            default_value='/r1/sdk_transport/motors_healthy',
        ),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('start_safety_supervisor',
                              default_value='true'),
        # Reading Unitree LowState still constructs a physical SDK/DDS
        # participant. A direct no-argument launch must therefore remain
        # completely detached from the robot; the staged project wrapper
        # opts in only after its read-only commissioning preflight.
        DeclareLaunchArgument('start_readonly_reader', default_value='false'),
        DeclareLaunchArgument('enable_head', default_value='false'),
        DeclareLaunchArgument('enable_arms', default_value='false'),
        DeclareLaunchArgument('enable_locomotion', default_value='false'),
        DeclareLaunchArgument('enable_prepare', default_value='false'),
        DeclareLaunchArgument(
            'head_absolute_envelope_confirmed', default_value='false'
        ),
        DeclareLaunchArgument('head_recenter_enabled', default_value='false'),
        DeclareLaunchArgument('head_recenter_confirmed', default_value='false'),
        DeclareLaunchArgument(
            'head_ownership_probe_only', default_value='true'
        ),
        DeclareLaunchArgument(
            'head_ownership_probe_confirmed', default_value='false'
        ),
        DeclareLaunchArgument('transport', default_value='mock'),
        DeclareLaunchArgument('send_commands', default_value='false'),
        DeclareLaunchArgument('commissioning_confirmed',
                              default_value='false'),
        DeclareLaunchArgument('commissioning_token', default_value=''),
        DeclareLaunchArgument('profile', default_value='slow-safe'),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=(
                '/home/unitree/Unitree_Project/Legacy_Robotics/robotics/'
                'ros2_ws/install/unitree_r1_description/share/'
                'unitree_r1_description/urdf/r1.urdf'
            ),
        ),
        DeclareLaunchArgument(
            'arm_topic',
            default_value='/r1_kinematics_control/debug/arm_trajectory',
        ),
    ]

    delayed = TimerAction(
        period=0.40,
        actions=[
            _include(
                'vr_teleop_bridge',
                'launch/vr_bridge.launch.py',
                {
                    'ros_localhost_only': '1',
                    'bind_address': LaunchConfiguration('udp_bind_address'),
                    'udp_port': LaunchConfiguration('udp_port'),
                    'allowed_source_ip': source_ip,
                },
                IfCondition(LaunchConfiguration('start_bridge')),
            ),
            _include(
                'r1_kinematics_control',
                'launch/r1_kinematics_control.launch.py',
                {
                    'use_sim_time': 'false',
                    'dry_run': 'true',
                    'simulation_mode': 'false',
                    'headset_relative_enabled': 'true',
                    'urdf_path': r1_urdf,
                    'joint_states_topic': '/r1/sdk/joint_states',
                    'arm_command_topic': arm_trajectory_topic,
                    'waist_hold_enabled': 'false',
                },
                IfCondition(LaunchConfiguration('enable_arms')),
            ),
            _include(
                'r1_hardware_adapter',
                'launch/r1_head_dry_run.launch.py',
                {'use_sim_time': 'false'},
                IfCondition(LaunchConfiguration('enable_head')),
            ),
            _include(
                'r1_hardware_adapter',
                'launch/r1_locomotion_dry_run.launch.py',
                {
                    'mode': LaunchConfiguration('profile'),
                    'input_topic': '/vr/cmd_vel',
                    'active_topic': '/vr/teleop/active',
                    'emergency_stop_topic': '/r1/safety/kill',
                },
                IfCondition(LaunchConfiguration('enable_locomotion')),
            ),
            _include(
                'r1_live_writer',
                'launch/r1_live_writer.launch.py',
                {
                    'transport': LaunchConfiguration('transport'),
                    'send_commands': LaunchConfiguration('send_commands'),
                    'enable_head': LaunchConfiguration('enable_head'),
                    'enable_arms': LaunchConfiguration('enable_arms'),
                    'enable_locomotion': LaunchConfiguration(
                        'enable_locomotion'
                    ),
                    'enable_prepare': LaunchConfiguration('enable_prepare'),
                    'head_absolute_envelope_confirmed': LaunchConfiguration(
                        'head_absolute_envelope_confirmed'
                    ),
                    'head_recenter_enabled': LaunchConfiguration(
                        'head_recenter_enabled'
                    ),
                    'head_recenter_confirmed': LaunchConfiguration(
                        'head_recenter_confirmed'
                    ),
                    'head_ownership_probe_only': LaunchConfiguration(
                        'head_ownership_probe_only'
                    ),
                    'head_ownership_probe_confirmed': LaunchConfiguration(
                        'head_ownership_probe_confirmed'
                    ),
                    'commissioning_confirmed': LaunchConfiguration(
                        'commissioning_confirmed'
                    ),
                    'commissioning_token': LaunchConfiguration(
                        'commissioning_token'
                    ),
                    'expected_vr_source_ip': source_ip,
                    'profile': LaunchConfiguration('profile'),
                    'arm_topic': arm_trajectory_topic,
                    'network_interface': interface,
                    'motor_health_topic': motor_health_topic,
                },
            ),
        ],
    )

    return LaunchDescription([
        *declarations,
        # r1_live_writer and r1_sdk_transport both link the vendor CycloneDDS
        # libraries at process load time, including mock/disabled modes.  Set
        # and validate one complete loader environment before the first child
        # launch is visited so a direct `ros2 launch` cannot recreate the
        # mixed ROS-libddsc/vendor-libddscxx crash.
        OpaqueFunction(function=_configure_physical_runtime),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        # ROS graph traffic remains local. The two vendor SDK processes use
        # only the explicitly selected robot Ethernet interface.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        _include(
            'r1_teleop_safety',
            'launch/r1_safety_supervisor.launch.py',
            condition=IfCondition(
                LaunchConfiguration('start_safety_supervisor')
            ),
        ),
        _include(
            'r1_sdk_transport',
            'launch/r1_sdk_transport.launch.py',
            {
                'sdk_enabled': start_readonly_reader,
                'network_interface': interface,
                'motor_health_topic': motor_health_topic,
            },
            IfCondition(start_readonly_reader),
        ),
        delayed,
    ])
