# `tello_ros`

`tello_ros` is a ROS2 driver for Tello and Tello EDU drones.

## Packages

There are 3 ROS2 packages:
* `tello_driver` is a C++ ROS2 node that connects to the drone
* `tello_msgs` is a set of ROS2 messages
* `tello_description` contains an URDF and other files useful for rviz2 and Gazebo

## Interface

Many Tello commands (e.g., `takeoff` and `land`) are long-running, and clients want to know when they complete.
In ROS1 it's common to use actions in these situations, but actions are not available in ROS2 Bouncy.
For now, the interface uses ROS a service `tello_command` to initiate commands,
and a corresponding ROS topic `tello_response` to indicate command completion.

Only one command may be active at a time.
When the command completes, the next command may be initiated.

Tello drones auto-land if no commands are received within 15 seconds.
The driver sends the appropriate keep-alive messages to avoid this.

The driver sends telemetry data on the `flight_data` topic.
The presence of this data is a good indication that the drone is connected.

The driver activates video and sends images on the `image_raw` topic.
Camera information is sent on the `camera_info` topic.

### Services

* `~tello_action` tello_msgs/TelloAction

### Published topics

* `~tello_response` [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)
* `~flight_data` tello_msgs/FlightData
* `~image_raw` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* `~camera_info` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

## Requirements

* Ubuntu 18.04
* ROS2 Crystal Clemmys, also install:
  * ros-crystal-cv-bridge
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
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 10 0 0 0'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'battery?'}"
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
source /opt/ros/crystal/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/ros2/flock2_ws/install/tello_msgs/lib
bash ~/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/183.4588.63/bin/clion.sh
~~~~

## Credits

The h264decoder is from: https://github.com/DaWelter/h264decoder

## Resources

* [Tello User Manual 1.4](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20User%20Manual%20v1.4.pdf). Explains the lights, among other things.
* [Tello firmware versions](https://dl-cdn.ryzerobotics.com/downloads/Tello/20180816/Tello_Release_Notes_en.pdf). Doesn't include Tello EDU firmware versions.
* [SDK 1.3](https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf). For Tello.
* [SDK 2.0](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf). For Tello EDU.
* [Tello EDU Mission Pad Guide (SDK 2.0)](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20Mission%20Pad%20User%20Guide.pdf) For Tello EDU.
