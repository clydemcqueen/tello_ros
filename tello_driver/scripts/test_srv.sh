#!/usr/bin/env bash

ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
sleep 10
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 20'}"
sleep 5
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 -20'}"
sleep 5
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 0'}"
sleep 1
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
