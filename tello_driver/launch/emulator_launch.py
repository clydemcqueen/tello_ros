from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


# Launch an emulator for testing


def generate_launch_description():
    emulator_path = 'install/tello_driver/lib/tello_driver/tello_emulator'
    tello_driver_params = [{'drone_ip': '127.0.0.1'}]

    return LaunchDescription([
        ExecuteProcess(cmd=[emulator_path], output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', node_name='tello_driver',
             parameters=tello_driver_params, output='screen'),
    ])
