#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/srv/tello_action.hpp"

namespace tello_joy {

// Simple teleop node:
// -- translate joystick commands to Tello actions and cmd_vel messages
// -- ignore all responses

// XBox One constants
constexpr int JOY_AXIS_LEFT_LR = 0;       // Left stick left/right; 1.0 is left and -1.0 is right
constexpr int JOY_AXIS_LEFT_FB = 1;       // Left stick forward/back; 1.0 is forward and -1.0 is back
constexpr int JOY_AXIS_RIGHT_LR = 3;      // Right stick left/right; 1.0 is left and -1.0 is right
constexpr int JOY_AXIS_RIGHT_FB = 4;      // Right stick forward/back; 1.0 is forward and -1.0 is back
constexpr int JOY_BUTTON_VIEW = 6;        // View button
constexpr int JOY_BUTTON_MENU = 7;        // Menu button

class TelloJoy : public rclcpp::Node
{
public:

  explicit TelloJoy() : Node("tello_joy")
  {
    using std::placeholders::_1;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", std::bind(&TelloJoy::joy_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    tello_client_ = create_client<tello_msgs::srv::TelloAction>("tello_action");

    (void)joy_sub_;
  }

  ~TelloJoy() {}

private:

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->buttons[joy_button_takeoff_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "takeoff";
      tello_client_->async_send_request(request);
    } else if (joy_msg->buttons[joy_button_land_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "land";
      tello_client_->async_send_request(request);
    } else {
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = joy_msg->axes[joy_axis_throttle_];
      twist_msg.linear.y = joy_msg->axes[joy_axis_strafe_];
      twist_msg.linear.z = joy_msg->axes[joy_axis_vertical_];
      twist_msg.angular.z = joy_msg->axes[joy_axis_yaw_];
      cmd_vel_pub_->publish(twist_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr tello_client_;

  // XBox One assignments
  const int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
  const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_button_takeoff_ = JOY_BUTTON_MENU;
  const int joy_button_land_ = JOY_BUTTON_VIEW;
};

} // namespace tello_joy

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<tello_joy::TelloJoy>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}