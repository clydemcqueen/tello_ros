import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch nodes required for joystick operation


def generate_launch_description():
    tello_gazebo_path = get_package_share_directory('tello_gazebo')
    map_path = os.path.join(tello_gazebo_path, 'worlds', 'f2_map.yaml')

    #fiducial_vlam_path = get_package_share_directory('fiducial_vlam')
    #tello_description_path = get_package_share_directory('tello_description')

    #map_path = os.path.join(fiducial_vlam_path, 'launch', 'fiducial_marker_locations_office.yaml')

    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', output='screen'),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,                               # Publish marker /tf
                'marker_length': 0.1778,                        # Marker length
                'marker_map_load_full_filename': map_path,      # Load a pre-built map from disk
                'make_not_use_map': 0}]),                       # Don't save a map to disk

        # Localize this drone against the map
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', parameters=[{
                'publish_tfs': 1,
                'base_frame_id': 'base_link',
                't_camera_base_x': 0.,
                't_camera_base_y': 0.,
                't_camera_base_z': -0.035,
                't_camera_base_roll': 0., # math.pi/2,
                't_camera_base_pitch': 0., # -math.pi/2,
                't_camera_base_yaw': 0.,
                'camera_frame_id': 'camera_link'}]),
    ])
