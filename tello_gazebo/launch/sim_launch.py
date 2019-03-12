"""Launch a simulation"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    world = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'tello.world')
    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', world], output='screen'),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
    ])
