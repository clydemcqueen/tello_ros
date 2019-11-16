#include "tello_joy_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/srv/tello_action.hpp"

namespace tello_joy
{

  TelloJoyNode::TelloJoyNode(const rclcpp::NodeOptions &options) :
    Node("tello_joy", options)
  {
    using std::placeholders::_1;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&TelloJoyNode::joy_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    tello_client_ = create_client<tello_msgs::srv::TelloAction>("tello_action");

    (void) joy_sub_;
  }

  TelloJoyNode::~TelloJoyNode()
  {}


  void TelloJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
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

} // namespace tello_joy

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_joy::TelloJoyNode)