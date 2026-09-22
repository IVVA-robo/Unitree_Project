import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_TRUE_VALUES = frozenset(('1', 'true', 'yes', 'on'))
_FALSE_VALUES = frozenset(('0', 'false', 'no', 'off', ''))


def _require_isolated_simulation(context):
    """Reject settings that could expose Gazebo command topics to a robot."""
    localhost = LaunchConfiguration('ros_localhost_only').perform(context)
    hardware = LaunchConfiguration('hardware_enabled').perform(context)
    if str(localhost).strip().lower() not in _TRUE_VALUES:
        raise RuntimeError(
            'telepresence_sim requires ros_localhost_only:=1; use a separate, '
            'reviewed bridge for any intentionally networked simulation.'
        )
    if str(hardware).strip().lower() not in _FALSE_VALUES:
        raise RuntimeError(
            'telepresence_sim requires hardware_enabled:=false and cannot '
            'publish to physical R1 hardware.'
        )
    return []


def generate_launch_description():
    description_share = get_package_share_directory('unitree_r1_description')
    description_launch = os.path.join(description_share, 'launch', 'gazebo.launch.py')
    urdf_path = os.path.join(description_share, 'urdf', 'r1.urdf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_localhost_only',
            default_value='1',
            description=(
                'Required safety boundary. Must remain 1 so simulation DDS '
                'traffic cannot reach a physical robot network.'
            ),
        ),
        DeclareLaunchArgument(
            'hardware_enabled',
            default_value='false',
            description='Required false; physical hardware is unsupported here.',
        ),
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument(
            'model_z',
            default_value='0.25',
            description=(
                'Extra Gazebo-only vertical offset in metres; increase it if the '
                'feet are hidden by the ground plane.'
            ),
        ),
        DeclareLaunchArgument('headset_relative_enabled', default_value='true'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '~/.ros/r1_telepresence_sim_body_calibration.json'
            ),
            description=(
                'Simulation-only body calibration. Keep this separate from the '
                'physical R1 calibration file.'
            ),
        ),
        OpaqueFunction(function=_require_isolated_simulation),
        # Do not merely default to localhost: force it after validating the
        # compatibility argument above.  This protects direct `ros2 launch`
        # users as well as the r1-sim wrapper.
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'model_z': LaunchConfiguration('model_z'),
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'torso_link', 'vr_tracking'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('r1_kinematics_control'),
                    'launch', 'r1_kinematics_control.launch.py',
                )
            ),
            # This launch owns only Gazebo controllers, so actuator output is
            # safe to enable inside the simulator. Standalone IK stays dry-run.
            launch_arguments={
                'urdf_path': urdf_path,
                'dry_run': 'false',
                'simulation_mode': 'true',
                'headset_relative_enabled': LaunchConfiguration(
                    'headset_relative_enabled'
                ),
                'calibration_file': LaunchConfiguration('calibration_file'),
                # IK remains the arm-pose source, while the whole-body planner
                # is the only node allowed to write the Gazebo arm controller.
                'arm_command_topic': '/r1/sim/ik_arm_trajectory',
                'waist_hold_enabled': 'false',
                # Simulation velocity intent is namespaced; it can never be
                # mistaken for a physical `/cmd_vel` writer.
                'cmd_vel_output_topic': '/r1/sim/safe_cmd_vel',
            }.items(),
        ),
        Node(
            package='r1_telepresence_sim',
            executable='r1_sim_locomotion_adapter',
            name='r1_sim_locomotion_adapter',
            parameters=[{
                'hardware_enabled': False,
                'input_topic': '/r1/sim/safe_cmd_vel',
                'output_topic': '/r1/sim/cmd_vel',
            }],
            output='screen',
        ),
        Node(
            package='r1_telepresence_sim',
            executable='r1_sim_leg_visualizer',
            name='r1_sim_leg_visualizer',
            parameters=[{
                'input_topic': '/r1/sim/cmd_vel',
                'active_topic': '/vr/teleop/active',
                # The whole-body planner owns the final controller topic.
                'output_topic': '/r1/sim/leg_trajectory',
                'model_name': 'unitree_r1',
            }],
            output='screen',
        ),
        Node(
            package='r1_telepresence_sim',
            executable='r1_sim_whole_body_planner',
            name='r1_sim_whole_body_planner',
            parameters=[{
                'cmd_topic': '/r1/sim/cmd_vel',
                'active_topic': '/vr/teleop/active',
                # leg_visualizer's debug JointState is the planner input.
                # Its controller output is a JointTrajectory; subscribing to
                # that same name would create a ROS topic type conflict and
                # leave the planner at neutral legs while arms still move.
                'leg_input_topic': '/r1/telepresence/leg_targets',
                'arm_input_topic': '/r1/sim/ik_arm_trajectory',
                'leg_output_topic': '/leg_trajectory_controller/joint_trajectory',
                'arm_output_topic': '/arm_trajectory_controller/joint_trajectory',
                'waist_output_topic': '/waist_hold_controller/commands',
                'debug_topic': '/r1/telepresence/whole_body_targets',
            }],
            output='screen',
        ),
        Node(
            package='r1_telepresence_sim',
            executable='r1_stereo_camera_viewer',
            name='r1_stereo_camera_viewer',
            parameters=[{'display': LaunchConfiguration('display')}],
            output='screen',
        ),
    ])
