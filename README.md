# `tello_ros`

`tello_ros` is a ROS2 driver for Tello and Tello EDU drones.

## Packages

There are 4 ROS packages:
* `tello_driver` is a C++ ROS node that connects to the drone
* `tello_msgs` is a set of ROS messages
* `tello_description` contains robot description (URDF) files
* `tello_gazebo` can be used to simulate a Tello drone in [Gazebo](http://gazebosim.org/),
 see the `README.md` in that package for details

## Interface

### Overview

The driver is designed to very simple while making it easy to integrate Tello drones into the ROS ecosystem.

The driver communicates with the drone using the Tello SDK, which has several advantages:
* The SDK is documented, and there's quite a bit of development activity around it, so it's likely to be stable.
* The SDK is text-based, which means that `tello_ros` can be simple but still provide full access to the SDK by passing
arbitrary strings to the drone.

Many Tello commands (e.g., `takeoff` and `land`) are long-running, and the drone returns `ok` or `error` on completion.
The driver provides the ROS service `tello_command` to initiate commands,
and the corresponding ROS topic `tello_response` to indicate command completion.

Per ROS convention, the driver also responds to `Twist` messages on the `cmd_vel` topic.
These are translated into `rc` commands and sent to the drone.
Velocity values are arbitrarily mapped from [-1.0, 1.0] to [-100, 100].
This may change in the future.

The driver parses telemetry data and sends it on the `flight_data` topic.
The presence of telemetry data is a good indicator that the drone is connected.

The driver parses the video stream and sends images on the `image_raw` topic.
Camera information is sent on the `camera_info` topic.

The Tello drones have a sophisticated visual odometry system and an onboard IMU, but there's minimal access 
to these internal systems. The driver does not publish odometry.

Additional notes:
* Only one command may be running at a time.
* If a command (other than `rc`) is currently running, incoming `cmd_vel` messages are ignored.
* Tello drones do not send responses for `rc` commands, and neither does the driver.
* The driver sends `command` and `streamon` commands at startup to initiate telemetry and video.
* If telemetry or video stops, the driver will attempt to restart by sending `command` and `streamon` commands.
* Roll (`Twist.angular.x`) and pitch (`Twist.angular.y`) are ignored in `cmd_vel` messages.
* The driver doesn't keep track of state, so it will happily send `rc` messages to the drone even if it's on the ground.
The drone just ignores them.
* You can send arbitrary strings to the drone via the `tello_command` service.
* Tello drones auto-land if no commands are received within 15 seconds.
The driver sends a `rc 0 0 0 0` command after 12 seconds of silence to avoid this.

### Services

* `~tello_action` tello_msgs/TelloAction

### Subscribed topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

### Published topics

* `~tello_response` [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)
* `~flight_data` tello_msgs/FlightData
* `~image_raw` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* `~camera_info` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

### Parameters

The defaults work well for a single Tello drone.

 Name         |  Description |  Default
--------------|--------------|----------
`drone_ip`    | Send commands to this IP address |  `192.168.10.1`
`drone_port`  | Send commands to this UDP port | `8889`
`command_port`| Send commands from this UDP port | `38065`
`data_port`   | Flight data (Tello state) will arrive on this UDP port  | `8890`
`video_port`  | Video data will arrive on this UDP port |  `11111`

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 20.04 box or VM.

Also install asio:
~~~
sudo apt install libasio-dev
~~~

### 2. Set up your ROS environment

[Install ROS2 Foxy](https://docs.ros.org/) with the `ros-foxy-desktop` option.

If you install binaries, be sure to also install the 
[development tools and ROS tools](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)
from the source installation instructions.

Install these additional packages:
~~~
sudo apt install ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers
~~~

### 3. Install `tello_ros`

Download, compile and install `tello_ros`:
~~~
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/foxy/setup.bash
# If you didn't intall Gazebo, skip tello_gazebo while building:
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo
~~~

## Teleop

The driver provides a simple launch file that will allow you to fly the drone using a wired XBox One gamepad.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and launch ROS:
~~~
cd ~/tello_ros_ws
source install/setup.bash
ros2 launch tello_driver teleop_launch.py
~~~

Hit the XBox One **menu** button to take off, and the **view** button to land.

If you don't have an XBox One gamepad, you can send commands using the ROS2 CLI:
~~~~
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 20'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'battery?'}"
~~~~

You can also send `cmd_vel` messages:
~~~~
ros2 topic pub /cmd_vel geometry_msgs/Twist  # Sends rc 0 0 0 0
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
~~~~

## Devices tested

* Tello
  * Firmware v01.04.35.01, SDK v1.3
* Tello EDU
  * Firmware v02.04.69.03, SDK v2.0

## Versions and branches

`tello_ros` was developed along with several other projects while ROS2 was rapidly changing.
All of the related projects adopted similar conventions around branch names:
* the `master` branch works with the latest ROS2 release (Foxy as of this writing)
* there may be branches for older ROS2 versions, such as `crystal`, `dashing` or `eloquent`

The following projects and branches were tested together:

* ROS2 Foxy with fiducial_vlam:
  * git clone https://github.com/ptrmu/ros2_shared.git
  * git clone https://github.com/ptrmu/fiducial_vlam.git
  * git clone https://github.com/clydemcqueen/tello_ros.git
  * git clone https://github.com/clydemcqueen/flock2.git

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

#### Tello SDK errata

* Tello drones do not respond to `rc` commands (the SDK suggests that they return `ok` or `error`)