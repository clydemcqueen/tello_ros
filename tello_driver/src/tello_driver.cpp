#include "tello_driver.hpp"

#include <iostream>

using asio::ip::udp;

namespace tello_driver {

constexpr int SPIN_RATE = 100;            // Message publish rate in Hz
constexpr int32_t STATE_TIMEOUT = 5;      // We stopped receiving telemetry
constexpr int32_t VIDEO_TIMEOUT = 5;      // We stopped receiving video
constexpr int32_t KEEP_ALIVE = 10;        // We stopped receiving input from other ROS nodes
constexpr int32_t COMMAND_TIMEOUT = 10;   // Drone didn't respond to a command

TelloDriver::TelloDriver() : Node("tello_driver")
{
  // ROS publishers
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  flight_data_pub_ = create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);
  tello_response_pub_ = create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 1);

  // ROS service
  command_srv_ = create_service<tello_msgs::srv::TelloAction>("tello_action",
    std::bind(&TelloDriver::command_callback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Sockets
  command_socket_ = std::make_unique<CommandSocket>(this);
  state_socket_ = std::make_unique<StateSocket>(this);
  video_socket_ = std::make_unique<VideoSocket>(this);
}

TelloDriver::~TelloDriver()
{
};

void TelloDriver::command_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
  std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
{
  (void)request_header;
  if (!state_socket_->receiving() || !video_socket_->receiving()) {
    RCLCPP_WARN(get_logger(), "Not connected, dropping '%s'", request->cmd.c_str());
    response->rc = response->ERROR_NOT_CONNECTED;
  } else if (command_socket_->waiting()) {
    RCLCPP_WARN(get_logger(), "Busy, dropping '%s'", request->cmd.c_str());
    response->rc = response->ERROR_BUSY;
  } else {
    command_socket_->initiate_command(request->cmd, true);
    response->rc = response->OK;
  }
}

// Do work at SPIN_RATE Hz
void TelloDriver::spin_once()
{
  static unsigned int counter = 0;
  counter++;

  if (counter % SPIN_RATE == 0) {
    spin_1s();
  }
}

// Do work every 1 second
void TelloDriver::spin_1s()
{
  //====
  // Startup
  //====

  if (!state_socket_->receiving() && !command_socket_->waiting()) {
    // First command to the drone must be "command"
    command_socket_->initiate_command("command", false);
    return;
  }

  if (state_socket_->receiving() && !video_socket_->receiving() && !command_socket_->waiting()) {
    // Start video
    command_socket_->initiate_command("streamon", false);
    return;
  }

  //====
  // Timeouts
  //====

  bool timeout = false;

  if (command_socket_->waiting() && now() - command_socket_->send_time() > rclcpp::Duration(COMMAND_TIMEOUT, 0)) {
    RCLCPP_ERROR(get_logger(), "Command timed out");
    command_socket_->timeout();
    timeout = true;
  }

  if (state_socket_->receiving() && now() - state_socket_->receive_time() > rclcpp::Duration(STATE_TIMEOUT, 0)) {
    RCLCPP_ERROR(get_logger(), "No state received for 5s");
    state_socket_->timeout();
    timeout = true;
  }

  if (video_socket_->receiving() && now() - video_socket_->receive_time() > rclcpp::Duration(VIDEO_TIMEOUT, 0)) {
    RCLCPP_ERROR(get_logger(), "No video received for 5s");
    video_socket_->timeout();
    timeout = true;
  }

  if (timeout) {
    return;
  }

  //====
  // Keep-alive, drone will auto-land if it hears nothing for 15s
  //====

  if (state_socket_->receiving() && video_socket_->receiving() && !command_socket_->waiting() &&
    now() - command_socket_->send_time() > rclcpp::Duration(KEEP_ALIVE, 0)) {
    command_socket_->initiate_command("rc 0 0 0 0", false);
    return;
  }
}

} // namespace tello_driver


int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::Rate r(tello_driver::SPIN_RATE);
  auto node = std::make_shared<tello_driver::TelloDriver>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  while (rclcpp::ok()) {
    // Do our work
    node->spin_once();

    // Respond to incoming ROS messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
