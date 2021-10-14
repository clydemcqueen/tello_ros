from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# Launch two emulators and two drivers for testing


def generate_launch_description():
    emulator_path = 'install/tello_driver/lib/tello_driver/tello_emulator'
    localhost = '127.0.0.1'

    dr1_cmd_port = 11001
    dr2_cmd_port = 11002

    em1_port = 12001
    em2_port = 12002

    dr1_data_port = 13001
    dr2_data_port = 13002

    dr1_video_port = 14001
    dr2_video_port = 14002

    dr1_params = [{
        'drone_ip': localhost,
        'drone_port': em1_port,
        'command_port': dr1_cmd_port,
        'data_port': dr1_data_port,
        'video_port': dr1_video_port
    }]

    dr2_params = [{
        'drone_ip': localhost,
        'drone_port': em2_port,
        'command_port': dr2_cmd_port,
        'data_port': dr2_data_port,
        'video_port': dr2_video_port
    }]

    return LaunchDescription([
        ExecuteProcess(cmd=[emulator_path, 'em1', str(em1_port), str(dr1_data_port), str(dr1_video_port)],
                       output='screen'),
        ExecuteProcess(cmd=[emulator_path, 'em2', str(em2_port), str(dr2_data_port), str(dr2_video_port)],
                       output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', node_name='dr1', namespace='dr1',
             parameters=dr1_params, output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', node_name='dr2', namespace='dr2',
             parameters=dr2_params, output='screen'),
    ])
