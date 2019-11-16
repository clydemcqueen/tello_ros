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
  // Note: this is a NOP, as there's only 1 node in this process
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);

  // Create and add joy node
  auto joy_node = std::make_shared<tello_joy::TelloJoyNode>(options);
  executor.add_node(joy_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
