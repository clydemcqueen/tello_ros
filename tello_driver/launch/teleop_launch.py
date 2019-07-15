from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy_main', output='screen'),
        Node(package='tello_driver', node_executable='tello_driver_main', output='screen'),
    ])
