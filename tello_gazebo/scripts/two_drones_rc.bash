# An example of how to control 2 drones using the ros2 CLI
# Uses "rc x y z yaw" to control velocity
# Do _not_ launch the joystick nodes

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
sleep 8

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'rc -0.2 0 0 0'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'rc -0.2 0 0 0'}"
sleep 2

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 0'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 0'}"
sleep 1

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'rc 0.2 0 0 0'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'rc 0.2 0 0 0'}"
sleep 2

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 0'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'rc 0 0 0 0'}"
sleep 1

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /drone2/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
