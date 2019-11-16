#include "tello_driver_node.hpp"

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

  // Create and add driver node
  auto node = std::make_shared<tello_driver::TelloDriverNode>(options);
  executor.add_node(node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
