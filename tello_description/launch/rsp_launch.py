import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Specify the name of the package and the urdf file
    pkg_name = 'tello_description'
    urdf_file_name = 'urdf/tello.urdf'

    # Use infp to process the file
    urdf = os.path.join(
        get_package_share_directory(pkg_name),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}] # add other parameters here if required
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher
    ])
