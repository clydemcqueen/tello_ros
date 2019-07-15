#include "tello_driver_node.hpp"

namespace tello_driver
{

  void TelloSocket::listen()
  {
    thread_ = std::thread(
      [this]()
      {
        for (;;) {
          size_t r = socket_.receive(asio::buffer(buffer_));
          process_packet(r);
        }
      });
  }

  bool TelloSocket::receiving()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receiving_;
  }

  rclcpp::Time TelloSocket::receive_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receive_time_;
  }

  void TelloSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;
  }

} // namespace tello_driver
