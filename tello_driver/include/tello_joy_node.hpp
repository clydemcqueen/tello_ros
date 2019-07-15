#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/srv/tello_action.hpp"

namespace tello_joy
{

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

  class TelloJoyNode : public rclcpp::Node
  {
  public:

    explicit TelloJoyNode(const rclcpp::NodeOptions &options);

    ~TelloJoyNode();

  private:

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

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
