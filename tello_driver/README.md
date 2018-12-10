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
ros2 topic pub -1 /command "std_msgs/String" "{data: 'battery?'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'wifi?'}"
ros2 topic pub -1 /command "std_msgs/String" "{data: 'takeoff'}"
~~~~

## SDK
 
### Supported versions

Which firmware versions? TODO

Which device versions? TODO

### SDK 1.3

Test results:
* video streaming works fine
* `takeoff` works fine
* `land` works fine
* `rc a b c d` works fine

### SDK 2.0

Test results:
* video streaming works fine
* `takeoff` always returns `error`, this is blocking most testing
* `land` always returns `ok`
* `streamon` and `streamoff` both return `ok`, but are ignored (the video is always streaming)

## Running nodes inside CLion

Set the LD_LIBRARY_PATH to pick up the compiled `tello_msgs` library.
Example:
~~~~
source /opt/ros/bouncy/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/ros2/flock2_ws/install/tello_msgs/lib
bash ~/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/183.4588.63/bin/clion.sh
~~~~

## h264decoder

From https://github.com/DaWelter/h264decoder.