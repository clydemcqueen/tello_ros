"""Publish static transforms"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello.urdf')

    return LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf]),
    ])
