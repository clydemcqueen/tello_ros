# tello_driver

tello_driver is a ROS2 driver for DJI Tello drones.

## Requirements

* Ubuntu 18.04
* ROS2 Bouncy
* libasio-dev 1.10.8-1
* ffmpeg 3.4.4-0ubuntu0.18.04.1, which includes these libraries:
  * libavcodec.so.57.107.100
  * libavutil.so.55.78.100
  * libswscale.so.4.8.100
* OpenCV 3

## Sending commands

Send arbitrary strings to the drone using the `command` topic.
Examples:
~~~~
ros2 topic pub -1 /command "std_msgs/String" "{data: 'command'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'streamon'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'battery?'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'wifi?'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'takeoff'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'land'}"
~~~~

## Devices tested

* Tello
  * Firmware v01.04.35.01, SDK v1.3
* Tello EDU
  * Firmware v02.04.69.03, SDK v2.0

## Running C++ nodes inside CLion

Set the LD_LIBRARY_PATH to pick up the compiled `tello_msgs` library.
Example:
~~~~
source /opt/ros/bouncy/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/ros2/flock2_ws/install/tello_msgs/lib
bash ~/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/183.4588.63/bin/clion.sh
~~~~

## Credits

The h264decoder is from: https://github.com/DaWelter/h264decoder

## Resources

[Tello firmware versions](https://dl-cdn.ryzerobotics.com/downloads/Tello/20180816/Tello_Release_Notes_en.pdf)