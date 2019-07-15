#include "tello_driver_node.hpp"
#include "tello_joy_node.hpp"

// Launch TelloDriver with use_intra_process_comms=true

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Use IPC
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);

  // Create and add driver node
  auto driver_node = std::make_shared<tello_driver::TelloDriverNode>(options);
  executor.add_node(driver_node);

  // Create and add joy node
  auto joy_node = std::make_shared<tello_joy::TelloJoyNode>(options);
  executor.add_node(joy_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
