from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch emulator for testing


def generate_launch_description():
    emulator_path = 'install/tello_driver/lib/tello_driver/tello_emulator'

    return LaunchDescription([
        ExecuteProcess(cmd=[emulator_path], output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', arguments=['127.0.0.1'], output='screen'),
    ])
