import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    config = os.path.join(get_package_share_directory('vr_teleop_bridge'), 'config', 'vr_bridge.yaml')
    return LaunchDescription([Node(package='vr_teleop_bridge', executable='vr_bridge', name='vr_teleop_bridge', output='screen', parameters=[config])])
