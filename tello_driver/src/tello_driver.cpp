#include "tello_driver.hpp"

#include <iostream>

using asio::ip::udp;

namespace tello_driver {

// Message publish rate in Hz
constexpr int SPIN_RATE = 100;

TelloDriver::TelloDriver() : Node("tello_driver")
{
  // ROS publishers
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
  flight_data_pub_ = create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);

  // ROS subscriptions
  using std::placeholders::_1;
  command_sub_ = create_subscription<std_msgs::msg::String>("command",std::bind(&TelloDriver::command_callback, this, _1));
  takeoff_sub_ = create_subscription<std_msgs::msg::Empty>("takeoff",std::bind(&TelloDriver::takeoff_callback, this, _1));
  land_sub_ = create_subscription<std_msgs::msg::Empty>("land", std::bind(&TelloDriver::land_callback, this, _1));
  flip_sub_ = create_subscription<tello_msgs::msg::Flip>("flip", std::bind(&TelloDriver::flip_callback, this, _1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",std::bind(&TelloDriver::cmd_vel_callback, this, _1));

  // Sockets
  command_socket_ = std::make_unique<CommandSocket>(this);
  state_socket_ = std::make_unique<StateSocket>(this);
  video_socket_ = std::make_unique<VideoSocket>(this);
}

TelloDriver::~TelloDriver()
{
};

bool TelloDriver::connected()
{
  return state_socket_->receiving() && video_socket_->receiving();
}

void TelloDriver::set_sdk(SDK sdk)
{
  static std::map<SDK, std::string> enum_names{
    {SDK::unknown, "unknown"},
    {SDK::v1_3, "v1.3"},
    {SDK::v2_0, "v2.0"}};

  if (sdk_ == SDK::unknown && sdk != SDK::unknown) {
    RCLCPP_INFO(get_logger(), "SDK version %s", enum_names[sdk].c_str());
    sdk_ = sdk;
  } else if (sdk_ != sdk) {
    RCLCPP_ERROR(get_logger(), "Getting conflicting SDK information");
  }
}

void TelloDriver::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (connected()) {
    command_socket_->send_command(msg->data);
  }
}

void TelloDriver::takeoff_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (connected()) {
    command_socket_->send_command("takeoff");
  }
}

void TelloDriver::land_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (connected()) {
    command_socket_->send_command("land");
  }
}

void TelloDriver::flip_callback(const tello_msgs::msg::Flip::SharedPtr msg)
{
  static std::map<uint8_t, std::string> direction_names{
    {tello_msgs::msg::Flip::FLIP_LEFT, "l"},
    {tello_msgs::msg::Flip::FLIP_RIGHT, "r"},
    {tello_msgs::msg::Flip::FLIP_FORWARD, "f"},
    {tello_msgs::msg::Flip::FLIP_BACK, "b"}};

  if (connected()) {
    std::stringstream ss;
    ss << "flip " << direction_names[msg->flip_command];
    command_socket_->send_command(ss.str());
  }
}

void TelloDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // TODO keep state, and don't send when we're on the ground, or in the middle of a command
  if (false /*connected()*/) {
    std::stringstream ss;
    ss << "rc " << msg->linear.y * -100.0 << " " << msg->linear.x * 100.0
      << " " << msg->linear.z * 100.0 << " " << msg->angular.z * -100.0;
    command_socket_->send_command(ss.str());
  }
}

// Do work at SPIN_RATE Hz
void TelloDriver::spin_once()
{
  mtx_.lock();

  static unsigned int counter = 0;
  counter++;

  if (counter % SPIN_RATE == 0) {
    spin_1s();
  }

  if (counter % (10 * SPIN_RATE) == 0) {
    spin_10s();
  }

  mtx_.unlock();
}

// Do work every 1 second
void TelloDriver::spin_1s()
{
  if (state_socket_->receiving() && now() - state_socket_->last_time() > rclcpp::Duration(5, 0)) {
    RCLCPP_ERROR(get_logger(), "No state received for 5s");
    state_socket_->reset();
  }

  if (video_socket_->receiving() && now() - video_socket_->last_time() > rclcpp::Duration(5, 0)) {
    RCLCPP_ERROR(get_logger(), "No video received for 5s");
    video_socket_->reset();
  }

  if (!state_socket_->receiving()) {
    // First command to the drone must be "command"
    command_socket_->send_command("command");
  }

  if (state_socket_->receiving() && !video_socket_->receiving()) {
    // Start video
    command_socket_->send_command("streamon");
  }
}

// Do work every 10 seconds
void TelloDriver::spin_10s()
{
  if (state_socket_->receiving() && video_socket_->receiving()) {
    // The drone will auto-land if it hears nothing for 15s
    command_socket_->send_command("battery?");
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
