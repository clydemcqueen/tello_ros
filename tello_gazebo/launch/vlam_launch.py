"""Simulate one or more Tello drones, using ArUco markers and fiducial_vlam for localization"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 1 or more drones:
    # drones = ['drone1', 'drone2']
    drones = ['drone1']

    tello_gazebo_path = get_package_share_directory('tello_gazebo')
    tello_description_path = get_package_share_directory('tello_description')

    world_path = os.path.join(tello_gazebo_path, 'worlds', 'fiducial.world')
    map_path = os.path.join(tello_gazebo_path, 'worlds', 'fiducial_map.yaml')

    # Global entities
    entities = [
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Load and publish a known map
        Node(package='fiducial_vlam', executable='vmap_main', output='screen',
             name='vmap_main', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0}]),  # Don't save a map to disk

        # Joystick driver, generates /namespace/joy messages
        # Only controls the first drone
        Node(package='joy', executable='joy_node', output='screen',
             namespace=drones[0]),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen',
             namespace=drones[0]),
    ]

    # Per-drone entities
    for idx, namespace in enumerate(drones):
        suffix = '_' + str(idx + 1)
        urdf_path = os.path.join(tello_description_path, 'urdf', 'tello' + suffix + '.urdf')

        entities.extend([
            # Add a drone to the simulation
            Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
                 arguments=[urdf_path, '0', str(idx), '1', '0']),

            # Localize this drone against the map
            Node(package='fiducial_vlam', executable='vloc_main', output='screen',
                 name='vloc_main', namespace=namespace, parameters=[{
                    'publish_tfs': 1,
                    'base_frame_id': 'base_link' + suffix,
                    't_camera_base_z': -0.035,
                    'camera_frame_id': 'camera_link' + suffix}]),

        ])

    return LaunchDescription(entities)
