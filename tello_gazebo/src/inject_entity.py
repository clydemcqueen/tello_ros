#!/usr/bin/env python

"""Inject an SDF or URDF file into Gazebo"""

import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def inject(xml: str, initial_pose: Pose):
    """Create a ROS node, and use it to call the SpawnEntity service"""

    rclpy.init()
    node = rclpy.create_node('inject_node')
    client = node.create_client(SpawnEntity, 'spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info('waiting for spawn_entity service...')
        client.wait_for_service()

    request = SpawnEntity.Request()
    request.xml = xml
    request.initial_pose = initial_pose
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()


if len(sys.argv) < 3:
    print('usage: ros2 run tello_gazebo inject_entity.py -- foo.urdf initial_z')
    sys.exit(1)

f = open(sys.argv[1], 'r')

p = Pose()
p.position.z = float(sys.argv[2])

inject(f.read(), p)
