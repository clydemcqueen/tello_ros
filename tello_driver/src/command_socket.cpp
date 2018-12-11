#include "tello_driver.hpp"

namespace tello_driver {

CommandSocket::CommandSocket(TelloDriver *driver) : TelloSocket(driver, 38065)
{
  buffer_ = std::vector<unsigned char>(1024);
  listen();
}

void CommandSocket::send_command(std::string command)
{
  RCLCPP_INFO(driver_->get_logger(), "Sending '%s'...", command.c_str());
  socket_.send_to(asio::buffer(command), remote_endpoint_);
}

void CommandSocket::process_packet(size_t r)
{
  last_time_ = driver_->now();

  if (!receiving_) {
    receiving_ = true;
  }

  RCLCPP_INFO(driver_->get_logger(), "Received '%s'", std::string(buffer_.begin(), buffer_.begin() + r).c_str());
}

} // namespace tello_driver
