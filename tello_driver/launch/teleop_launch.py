from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', output='screen'),
    ])
