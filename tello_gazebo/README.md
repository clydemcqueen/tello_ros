## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing and very simple flight dynamics
* `markers` contains Gazebo models for fiducial markers
* `fiducial.world` is a simple world with a bunch of fiducial markers
* `inject_entity.py` is a script that will read an URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera

As of this writing ROS2 Crystal + Gazebo v9 integration is still developing. YMMV.

Install Gazebo v9:

    sudo apt install gazebo9 libgazebo9 libgazebo9-dev

Install these additional ROS packages:

    sudo apt install ros-crystal-gazebo-ros-pkgs ros-crystal-cv-bridge

Run a teleop simulation:

    cd ~/tello_ros_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    ros2 launch tello_gazebo simple_launch.py

If you run into the "No namespace found" error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models

If you run into a dynamic linking problem ("libCameraPlugin.so: cannot open shared object file...") try this workaround:

    cd ~/tello_ros_ws/src
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
    cd gazebo_ros_pkgs
    git checkout 9593afce820fd290cb0b0c44dffd4e04d927251a
    cd ~/tello_ros_ws
    colcon build --event-handlers console_direct+
    source install/local_setup.bash
    export GAZEBO_PLUGIN_PATH=${PWD}/install/gazebo_plugins/lib
    cp /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/* install/gazebo_plugins/lib
    ros2 launch tello_gazebo simple_launch.py
    
Integrate with `fiducial_vlam`:

    cd ~/tello_ros_ws/src
    git clone https://github.com/ptrmu/fiducial_vlam.git
    git clone https://github.com/clydemcqueen/odom_filter.git
    cd ..      
    colcon build --event-handlers console_direct+
    source install/local_setup.bash
    ros2 launch tello_gazebo vlam_launch.py
