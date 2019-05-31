#!/usr/bin/env python

"""
Build Gazebo world and fiducial_vlam map files from a list of markers and poses
Usage:
    cd src/tello_ros/tello_gazebo/worlds
    build_world.py

Marker format: [marker_num, x, y, z, roll, pitch, yaw]

Transformation notation:
   t_destination_source is a transform
   vector_destination = t_destination_source * vector_source
   xxx_f_destination means xxx is expressed in destination frame
   xxx_pose_f_destination is equivalent to t_destination_xxx
   t_a_c = t_a_b * t_b_c

Also:
   r_destination_source is a fixed axis rotation per https://www.ros.org/reps/rep-0103.html
"""

import math
import transformations as xf

pi = math.pi
pi2 = math.pi / 2

# SDF and fiducial_vlam have different coordinate models
t_world_map = xf.quaternion_matrix([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])


def build_world(name, markers):
    world_file = open(name, 'w')
    world_file.write("""<?xml version="1.0"?>

<sdf version="1.6">
  <world name="default">

    <!-- Tweak to run at higher or lower speeds -->
    <physics name="normal" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- ArUco markers -->
""")
    for marker in markers:
        world_file.write(f"""    <model name="marker{marker[0]}">
      <include>
        <static>true</static>
        <uri>model://marker_{marker[0]}</uri>
      </include>
      <pose>{marker[1]} {marker[2]} {marker[3]} {marker[4]} {marker[5]} {marker[6]}</pose>
    </model>
""")
    world_file.write("""
  </world>
</sdf>""")
    world_file.close()


def build_map(name, markers):
    map_file = open(name, 'w')
    map_file.write("""# Map for orca.world
# All marker locations are fixed (f: 1)

marker_length: 0.1778
markers:
""")
    for marker in markers:
        t_marker_world = xf.euler_matrix(marker[4], marker[5], marker[6])
        t_marker_map = t_marker_world @ t_world_map
        r_marker_map = xf.euler_from_matrix(t_marker_map)

        map_file.write(f"""  - id: {marker[0]}
    u: 1
    f: 1
    xyz: [{marker[1]}, {marker[2]}, {marker[3]}]
    rpy: [{r_marker_map[0]}, {r_marker_map[1]}, {r_marker_map[2]}]
""")
    map_file.close()


fiducial = [
    [0, 2, 0, 0.5, 0, -pi2, 0],
    [1, 2, 1, 0.5, 0, -pi2, 0],
    [2, 2, 2, 0.5, 0, -pi2, 0],
    [3, 2, 3, 0.5, 0, -pi2, 0],
    [4, 2, 0, 1.5, 0, -pi2, 0],
    [5, 2, 1, 1.5, 0, -pi2, 0],
    [6, 2, 2, 1.5, 0, -pi2, 0],
    [7, 2, 3, 1.5, 0, -pi2, 0],
]

f2 = [
    [0, 0.5, 1, 1, 0, -pi2, 0],
    [1, 0.5, 2, 1, 0, -pi2, 0],
    [2, -1, -0.5, 1, 0, -pi2, -pi2],
    [3, -2, -0.5, 1, 0, -pi2, -pi2],
    [4, -1, 3.5, 1, 0, -pi2, pi2],
    [5, -2, 3.5, 1, 0, -pi2, pi2],
    [6, -3.5, 2, 1, 0, -pi2, pi],
    [7, -3.5, 1, 1, 0, -pi2, pi],
]

worlds = [
    ['fiducial.world', 'fiducial_map.yaml', fiducial],
    ['f2.world', 'f2_map.yaml', f2],
]

for world in worlds:
    build_world(world[0], world[2])
    build_map(world[1], world[2])
