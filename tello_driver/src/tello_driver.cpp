#include "tello_driver.hpp"

using asio::ip::udp;

namespace tello_driver
{

  constexpr int SPIN_RATE = 100;            // Spin rate in Hz
  constexpr int32_t STATE_TIMEOUT = 4;      // We stopped receiving telemetry
  constexpr int32_t VIDEO_TIMEOUT = 4;      // We stopped receiving video
  constexpr int32_t KEEP_ALIVE = 12;        // We stopped receiving input from other ROS nodes
  constexpr int32_t COMMAND_TIMEOUT = 9;    // Drone didn't respond to a command

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
                                                                          std::placeholders::_1, std::placeholders::_2,
                                                                          std::placeholders::_3));

    // ROS subscription
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                  std::bind(&TelloDriver::cmd_vel_callback, this,
                                                                            std::placeholders::_1));

    // Parameters
    std::string drone_ip;
    int drone_port;
    int command_port;
    int data_port;
    int video_port;

    get_parameter_or("drone_ip", drone_ip, std::string("192.168.10.1"));
    get_parameter_or("drone_port", drone_port, 8889);

    get_parameter_or("command_port", command_port, 38065);
    get_parameter_or("data_port", data_port, 8890);
    get_parameter_or("video_port", video_port, 11111);

    RCLCPP_INFO(get_logger(), "Drone at %s:%d", drone_ip.c_str(), drone_port);
    RCLCPP_INFO(get_logger(), "Listening for command responses on localhost:%d", command_port);
    RCLCPP_INFO(get_logger(), "Listening for data on localhost:%d", data_port);
    RCLCPP_INFO(get_logger(), "Listening for video on localhost:%d", video_port);

    // Sockets
    command_socket_ = std::make_unique<CommandSocket>(this, drone_ip, drone_port, command_port);
    state_socket_ = std::make_unique<StateSocket>(this, data_port);
    video_socket_ = std::make_unique<VideoSocket>(this, video_port);
  }

  TelloDriver::~TelloDriver()
  {
  };

  void TelloDriver::command_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
    std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
  {
    (void) request_header;
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

  void TelloDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // TODO cmd_vel should specify velocity, not joystick position
    if (!command_socket_->waiting()) {
      std::ostringstream rc;
      rc << "rc " << static_cast<int>(round(msg->linear.y * -100))
         << " " << static_cast<int>(round(msg->linear.x * 100))
         << " " << static_cast<int>(round(msg->linear.z * 100))
         << " " << static_cast<int>(round(msg->angular.z * -100));
      command_socket_->initiate_command(rc.str(), false);
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
