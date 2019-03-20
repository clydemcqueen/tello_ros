# An example of how to control 2 drones using the ros2 CLI
# Uses /cmd_vel to control velocity
# Do _not_ launch the joystick nodes

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
sleep 8

ros2 topic pub --once /drone1/cmd_vel geometry_msgs/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /drone2/cmd_vel geometry_msgs/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

ros2 topic pub --once /drone1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /drone2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

ros2 topic pub --once /drone1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /drone2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

ros2 topic pub --once /drone1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /drone2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
