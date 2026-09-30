import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('r1_kinematics_control')
    config = os.path.join(share, 'config', 'r1_control.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path = LaunchConfiguration('urdf_path')
    dry_run = LaunchConfiguration('dry_run')
    simulation_mode = LaunchConfiguration('simulation_mode')
    headset_relative = LaunchConfiguration('headset_relative_enabled')
    calibration_file = LaunchConfiguration('calibration_file')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')
    arm_command_topic = LaunchConfiguration('arm_command_topic')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    waist_hold_enabled = LaunchConfiguration('waist_hold_enabled')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('response_config', default_value=config),
        DeclareLaunchArgument('exhibition_session_mode', default_value='false'),
        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='Suppress actuator/controller outputs and publish debug only',
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            description=(
                'Permit non-dry-run controller topics only for an isolated '
                'Gazebo launch; never use this for physical hardware.'
            ),
        ),
        DeclareLaunchArgument('headset_relative_enabled', default_value='true'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '~/Unitree_Project/Configs/Calibration/'
                'r1_body_calibration.json'
            ),
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value='',
            description='Optional absolute path to an already expanded R1 URDF',
        ),
        DeclareLaunchArgument(
            'cmd_vel_output_topic',
            default_value='/vr/safe_cmd_vel',
            description=(
                'Final safe velocity intent topic. Gazebo overrides this with '
                'a simulation namespace; standalone dry-run publishes debug only.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_command_topic',
            default_value='/arm_trajectory_controller/joint_trajectory',
            description=(
                'Arm controller input. The simulation whole-body planner may '
                'use a private staging topic here.'
            ),
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='JointState feedback used to seed and stabilize IK',
        ),
        DeclareLaunchArgument(
            'waist_hold_enabled',
            default_value='true',
            description=(
                'Publish the neutral waist hold directly. Disable when a '
                'simulation whole-body planner owns the waist controller.'
            ),
        ),
        DeclareLaunchArgument(
            'shoulder_height_offset_m',
            default_value='0.0',
            description='Shoulder height offset captured by the next calibration',
        ),
        DeclareLaunchArgument(
            'shoulder_forward_offset_m',
            default_value='-0.02',
            description='Shoulder fore/aft offset captured by the next calibration',
        ),
        DeclareLaunchArgument(
            'shoulder_width_m',
            default_value='0.40',
            description='Operator shoulder width captured by the next calibration',
        ),
        DeclareLaunchArgument(
            'arm_motion_scale',
            default_value='1.15',
            description='Live headset-relative hand-delta scale',
        ),
        DeclareLaunchArgument('max_forward_mps', default_value='0.35'),
        DeclareLaunchArgument('max_lateral_mps', default_value='0.25'),
        DeclareLaunchArgument('max_yaw_rps', default_value='0.60'),
        Node(
            package='r1_kinematics_control',
            executable='r1_kinematics_control',
            name='r1_kinematics_control',
            output='screen',
            parameters=[
                config,
                LaunchConfiguration('response_config'),
                {
                    'exhibition_session_mode': ParameterValue(
                        LaunchConfiguration('exhibition_session_mode'), value_type=bool
                    ),
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'urdf_path': urdf_path,
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'simulation_mode': ParameterValue(
                        simulation_mode, value_type=bool
                    ),
                    'headset_relative_enabled': ParameterValue(
                        headset_relative, value_type=bool
                    ),
                    'body_proxy.calibration_file': calibration_file,
                    'body_proxy.shoulder_height_offset_m': ParameterValue(
                        LaunchConfiguration('shoulder_height_offset_m'),
                        value_type=float,
                    ),
                    'body_proxy.shoulder_forward_offset_m': ParameterValue(
                        LaunchConfiguration('shoulder_forward_offset_m'),
                        value_type=float,
                    ),
                    'body_proxy.shoulder_width_m': ParameterValue(
                        LaunchConfiguration('shoulder_width_m'),
                        value_type=float,
                    ),
                    'body_proxy.motion_scale': ParameterValue(
                        LaunchConfiguration('arm_motion_scale'), value_type=float
                    ),
                    'max_forward_mps': ParameterValue(
                        LaunchConfiguration('max_forward_mps'), value_type=float
                    ),
                    'max_lateral_mps': ParameterValue(
                        LaunchConfiguration('max_lateral_mps'), value_type=float
                    ),
                    'max_yaw_rps': ParameterValue(
                        LaunchConfiguration('max_yaw_rps'), value_type=float
                    ),
                    'cmd_vel_output_topic': cmd_vel_output_topic,
                    'arm_command_topic': arm_command_topic,
                    'joint_states_topic': joint_states_topic,
                    'waist_hold_enabled': ParameterValue(
                        waist_hold_enabled, value_type=bool
                    ),
                },
            ],
        ),
    ])
