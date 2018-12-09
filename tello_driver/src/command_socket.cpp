#include "tello_driver.hpp"

#include <iostream>

namespace tello_driver {

CommandSocket::CommandSocket(TelloDriver *driver) : TelloSocket(driver, 0)
{
  buffer_ = std::vector<unsigned char>(1024);
}

void CommandSocket::send_command(std::string command)
{
  socket_.send_to(asio::buffer(command), remote_endpoint_);

  // The first send on the command socket will bind the local side to a port
  if (!local_endpoint_bound_) {
    local_endpoint_bound_ = true;

    // Listen for command responses from the drone
    thread_ = std::thread(
      [this]()
      {
        for (;;) {
          size_t r = socket_.receive(asio::buffer(buffer_));
          process_packet(r);
        }
      });
  }
}

void CommandSocket::process_packet(size_t r)
{
  last_time_ = driver_->now();

  if (!receiving_) {
    driver_->lock();
    std::cout << "Receiving command responses! " << r << std::endl;
    driver_->unlock();
    receiving_ = true;
  }
}

} // namespace tello_driver
