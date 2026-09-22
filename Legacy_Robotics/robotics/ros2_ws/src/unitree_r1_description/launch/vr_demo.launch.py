import os, glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_r1_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'r1.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Умный поиск файла настроек RViz
    rviz_files = glob.glob(os.path.join(pkg_share, '**', '*.rviz'), recursive=True)
    rviz_args = ['-d', rviz_files[0]] if rviz_files else []

    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_desc}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'unitree_r1', '-topic', 'robot_description']),
        Node(package='rviz2', executable='rviz2', arguments=rviz_args)
    ])
