## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing and very simple flight dynamics
* `markers` contains Gazebo models for fiducial markers
* `fiducial.world` is a simple world with a bunch of fiducial markers
* `inject_entity.py` is a script that will read an URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera

#### Python

Add transformations.py v2018.9.5 to your Python environment.

#### Install Gazebo v9

    sudo apt install gazebo11 libgazebo11 libgazebo11-dev
    
Run `gazebo` on the command line, fix any problems before continuing.

#### Additional ROS packages

    sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cv-bridge

#### Run a teleop simulation

    cd ~/tello_ros_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py
    
You will see a single drone in a blank world.
You can control the drone using the joystick.

If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    
####  Integrate with `fiducial_vlam`

    cd ~/tello_ros_ws/src
    git clone https://github.com/ptrmu/fiducial_vlam.git
    cd ..      
    colcon build --event-handlers console_direct+
    source install/local_setup.bash
    ros2 launch tello_gazebo vlam_launch.py

You'll see 2 drones appear facing a field of ArUco markers.
Both drones will be localized against the markers -- run rviz2 to see the results.
You can only control drone1.