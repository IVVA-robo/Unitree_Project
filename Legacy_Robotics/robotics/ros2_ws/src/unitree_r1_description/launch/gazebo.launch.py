import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_r1_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'r1.urdf')
    model_z = LaunchConfiguration('model_z')

    # Чтение файла URDF для передачи в систему ROS 2
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Запуск пустого мира Gazebo
    start_gazebo = ExecuteProcess(
        # libgazebo_ros_init publishes /clock and initializes the ROS node;
        # without it nodes using use_sim_time never receive timer callbacks.
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # Publish TF and /robot_description for both Gazebo and the IK node.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    # 2. Спавн робота
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'unitree_r1', '-topic', 'robot_description', '-z', model_z],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # This controller is intentionally simulation-only.  The physical R1
    # driver has its own low-level locomotion path and never uses this launch.
    leg_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    waist_hold_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['waist_hold_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # The gazebo_ros2_control manager appears while the entity is spawned.
    # Start spawners only after spawn_entity exits successfully, with a small
    # grace period for the controller manager service to become available.
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster,
                        waist_hold_controller,
                        arm_trajectory_controller,
                        leg_trajectory_controller,
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_z',
            default_value='0.25',
            description=(
                'Additional simulation-only vertical offset in metres.  Raise '
                'the proxy when the feet are hidden by the ground plane.'
            ),
        ),
        start_gazebo,
        robot_state_publisher,
        spawn_entity,
        load_controllers,
    ])
