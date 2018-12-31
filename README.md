# `tello_ros`

`tello_ros` is a ROS2 driver for Tello and Tello EDU drones.

## Packages

There are 2 ROS2 packages:
* `tello_driver` is a C++ ROS2 node that connects to the drone
* `tello_msgs` is a set of ROS2 messages

## Interface

The driver is designed to very simple while making it easy to integrate Tello drones into the ROS ecosystem.

The driver communicates with the drone using the Tello SDK, which has several advantages:
* The SDK is documented, and there's quite a bit of development activity around it, so it's likely to be stable.
* The SDK is text-based, which means that `tello_ros` can be simple but still provide full access to the SDK by passing
arbitrary strings to the drone.

Many Tello commands (e.g., `takeoff` and `land`) are long-running, and the drone returns `ok` or `error` on completion.
In ROS it's common to use actions in these situations, but actions are not available in `rclpy`
(the ROS2 Python client) as of ROS2-Crystal.
For now, the driver provides a ROS service `tello_command` to initiate commands,
and a corresponding ROS topic `tello_response` to indicate command completion.
This will likely change when ROS2-Dashing is released.

Per ROS convention, the driver also responds to `Twist` messages on the `cmd_vel` topic.
These are translated into `rc` commands and sent to the drone.
Velocity values are arbitrarily mapped from [-1.0, 1.0] to [-100, 100].
This may change in the future.

The driver parses telemetry data and sends it on the `flight_data` topic.
The presence of telemetry data is a good indicator that the drone is connected.

The driver parses the video stream and sends images on the `image_raw` topic.
Camera information is sent on the `camera_info` topic.

The Tello drones have a sophisticated visual odometry system and an onboard IMU, but there's minimal SDK access 
to this information. The driver doesn't publish odometry.
Clients may try to interpret the telemetry or video data to generate odometry.

Additional notes:
* Only one command may be running at a time.
* If a command (other than `rc`) is currently running, incoming `cmd_vel` messages are ignored.
* Drones do not send responses for `rc` commands, and neither does the driver.
* The driver sends `command` and `streamon` commands at startup to initiate telemetry and video.
* If telemetry or video stops, the driver will attempt to restart by sending `command` and `streamon` commands.
* Roll (`Twist.angular.x`) and pitch (`Twist.angular.y`) are ignored in `cmd_vel` messages.
* The driver doesn't keep track of state, so it will happily send `rc` messages to the drone even if it's on the ground.
The drone just ignores them.
* You can send arbitrary strings to the drone via the `tello_command` service.
* Tello drones auto-land if no commands are received within 15 seconds.
The driver sends an `rc 0 0 0 0` command after 10 seconds of silence to avoid this.

### Services

* `~tello_action` tello_msgs/TelloAction

### Subscribed topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

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

Send commands (arbitrary strings) to the drone using the `tello_action` service.
Examples:
~~~~
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 10 0 0 0'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'battery?'}"
~~~~

Send `rc` commands to the drone using the `cmd_vel` topic.
Roll and pitch are ignored.
Examples:
~~~~
ros2 topic pub /cmd_vel geometry_msgs/Twist  # Sends rc 0 0 0 0
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
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

* [Tello User Manual 1.4](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20User%20Manual%20v1.4.pdf)
* [SDK 1.3](https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf)
for Tello, see the errata below
* [SDK 2.0](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
for Tello EDU, see the errata below
* [Tello EDU Mission Pad Guide (SDK 2.0)](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20Mission%20Pad%20User%20Guide.pdf)
for Tello EDU
* [Tello Pilots Developer Forum](https://tellopilots.com/forums/tello-development.8/)
is a good developer community

## Tello SDK errata

* Drones do not respond to `rc` commands (SDK 1.3, 2.0)
