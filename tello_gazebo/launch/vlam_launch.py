"""Simulate a Tello drone, using ArUco markers and fiducial_vlam for localization"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = 'solo'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'aruco.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello.urdf')

    return LaunchDescription([
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '1']),

        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Fire up a joystick
        Node(package='joy', node_executable='joy_node', output='screen'),

        # Very simple joystick driver, will map /joy messages to /cmd_vel, etc.
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),

        # Build a map using ArUco markers
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vloc_node', parameters=[{'marker_length': 0.1778}]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=ns, parameters=[{
                'publish_tfs': 0,
                'camera_frame_id': 'camera_frame'}]),

        # Kalman filter
        Node(package='flock2', node_executable='filter_node', output='screen',
             node_name='filter_node', node_namespace=ns),
    ])
