#include "tello_driver_node.hpp"

namespace tello_driver
{

  CommandSocket::CommandSocket(TelloDriverNode *driver, std::string drone_ip,
                               unsigned short drone_port, unsigned short command_port) :
    TelloSocket(driver, command_port),
    remote_endpoint_(asio::ip::address_v4::from_string(drone_ip), drone_port),
    send_time_(rclcpp::Time(0L, RCL_ROS_TIME))
  {
    buffer_ = std::vector<unsigned char>(1024);
    listen();
  }

  void CommandSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;

    if (waiting_) {
      complete_command(tello_msgs::msg::TelloResponse::TIMEOUT, "error: command timed out");
    }
  }

  bool CommandSocket::waiting()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return waiting_;
  }

  rclcpp::Time CommandSocket::send_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return send_time_;
  }

  void CommandSocket::initiate_command(std::string command, bool respond)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!waiting_) {
      RCLCPP_DEBUG(driver_->get_logger(), "Sending '%s'...", command.c_str());
      socket_.send_to(asio::buffer(command), remote_endpoint_);
      send_time_ = driver_->now();

      // Wait for a response for all commands except "rc"
      if (command.rfind("rc", 0) != 0) {
        respond_ = respond;
        waiting_ = true;
      }
    }
  }

  void CommandSocket::complete_command(uint8_t rc, std::string str)
  {
    if (respond_) {
      tello_msgs::msg::TelloResponse response_msg;
      response_msg.rc = rc;
      response_msg.str = str;
      driver_->tello_response_pub_->publish(response_msg);
    }
    waiting_ = false;
  }

  void CommandSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    receive_time_ = driver_->now();

    if (!receiving_) {
      receiving_ = true;
    }

    std::string str = std::string(buffer_.begin(), buffer_.begin() + r);
    if (waiting_) {
      RCLCPP_DEBUG(driver_->get_logger(), "Received '%s'", str.c_str());
      complete_command(str == "error" ? tello_msgs::msg::TelloResponse::ERROR : tello_msgs::msg::TelloResponse::OK,
                       str);
    } else {
      RCLCPP_WARN(driver_->get_logger(), "Unexpected '%s'", str.c_str());
    }
  }

} // namespace tello_driver
