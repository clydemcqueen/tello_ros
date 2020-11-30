#include "tello_driver_node.hpp"

#include "ros2_shared/context_macros.hpp"

using asio::ip::udp;

namespace tello_driver
{

#define TELLO_DRIVER_ALL_PARAMS \
  CXT_MACRO_MEMBER(               /* Send commands to this IP address */ \
  drone_ip, \
  std::string, std::string("192.168.10.1")) \
  CXT_MACRO_MEMBER(               /* Send commands to this port */ \
  drone_port, \
  int, 8889) \
  CXT_MACRO_MEMBER(               /* Send commands from this port */ \
  command_port, \
  int, 38065) \
  CXT_MACRO_MEMBER(               /* Flight data will arrive at this port */ \
  data_port, \
  int, 8890) \
  CXT_MACRO_MEMBER(               /* Video data will arrive at this port */ \
  video_port, \
  int, 11111) \
  CXT_MACRO_MEMBER(               /* Camera calibration path */ \
  camera_info_path, \
  std::string, "install/tello_driver/share/tello_driver/cfg/camera_info.yaml") \
  /* End of list */

  struct TelloDriverContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CXT_MACRO_DEFINE_MEMBERS(TELLO_DRIVER_ALL_PARAMS)
  };

  constexpr int32_t STATE_TIMEOUT = 4;      // We stopped receiving telemetry
  constexpr int32_t VIDEO_TIMEOUT = 4;      // We stopped receiving video
  constexpr int32_t KEEP_ALIVE = 12;        // We stopped receiving input from other ROS nodes
  constexpr int32_t COMMAND_TIMEOUT = 9;    // Drone didn't respond to a command

  TelloDriverNode::TelloDriverNode(const rclcpp::NodeOptions &options) :
    Node("tello_driver", options)
  {
    // ROS publishers
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    flight_data_pub_ = create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);
    tello_response_pub_ = create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 1);

    // ROS service
    command_srv_ = create_service<tello_msgs::srv::TelloAction>(
      "tello_action", std::bind(&TelloDriverNode::command_callback, this,
                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // ROS subscription
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&TelloDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // ROS timer
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(1s, std::bind(&TelloDriverNode::timer_callback, this));

    // Parameters - Allocate the parameter context as a local variable because it is not used outside this routine
    TelloDriverContext cxt{};
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(TELLO_DRIVER_ALL_PARAMS, [this]()
    {})

    // NOTE: This is not setup to dynamically update parameters after ths node is running.

    RCLCPP_INFO(get_logger(), "Drone at %s:%d", cxt.drone_ip_.c_str(), cxt.drone_port_);
    RCLCPP_INFO(get_logger(), "Listening for command responses on localhost:%d", cxt.command_port_);
    RCLCPP_INFO(get_logger(), "Listening for data on localhost:%d", cxt.data_port_);
    RCLCPP_INFO(get_logger(), "Listening for video on localhost:%d", cxt.video_port_);

    // Sockets
    command_socket_ = std::make_unique<CommandSocket>(this, cxt.drone_ip_, cxt.drone_port_, cxt.command_port_);
    state_socket_ = std::make_unique<StateSocket>(this, cxt.data_port_);
    video_socket_ = std::make_unique<VideoSocket>(this, cxt.video_port_, cxt.camera_info_path_);
  }

  TelloDriverNode::~TelloDriverNode()
  {
  }

  void TelloDriverNode::command_callback(
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

  void TelloDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
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

  // Do work every second
  void TelloDriverNode::timer_callback()
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

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_driver::TelloDriverNode)