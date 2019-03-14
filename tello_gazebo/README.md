## Running a simulation in Gazebo (WIP)

Install Gazebo v9:

    sudo apt install gazebo9 libgazebo9 libgazebo9-dev

Install these additional ROS packages:

    sudo apt install ros-crystal-gazebo-ros-pkgs ros-crystal-cv-bridge

Running a simulation:

    cd ~/tello_ros_ws
    source /opt/ros/crystal/setup.bash
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    ros2 launch tello_gazebo sim_launch.py

If you run into a dynamic linking problem ("libCameraPlugin.so: cannot open shared object file...") try this workaround:

    cd ~/tello_ros_ws/src
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
    cd gazebo_ros_pkgs
    git checkout ros2
    cd ~/tello_ros_ws
    colcon build --event-handlers console_direct+
    source install/setup.bash
    export GAZEBO_PLUGIN_PATH=${PWD}/install/gazebo_plugins/lib
    cp /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/* install/gazebo_plugins/lib
    ros2 launch tello_gazebo sim_launch.py