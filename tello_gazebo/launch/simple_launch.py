"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello.urdf')

    return LaunchDescription([
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1']),

        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Fire up a joystick
        Node(package='joy', node_executable='joy_node', output='screen'),

        # Very simple joystick driver, will map /joy messages to /cmd_vel, etc.
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
    ])
