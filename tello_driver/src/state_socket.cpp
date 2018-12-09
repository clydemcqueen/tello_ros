#include "tello_driver.hpp"

#include <iostream>
#include <regex>

namespace tello_driver {

// v1.3: pitch:0;roll:-1;yaw:0;vgx:0;vgy:0;vgz:0;templ:46;temph:49;tof:10;h:0;bat:100;baro:18.92;time:0;agx:-12.00;agy:16.00;agz:-993.00;
// v2.0: mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;roll:5;yaw:0;vgx:0;vgy:0;vgz:0;templ:44;temph:47;tof:10;h:0;bat:100;baro:17.84;time:0;agx:10.00;agy:-98.00;agz:-988.00;

StateSocket::StateSocket(TelloDriver *driver) : TelloSocket(driver, 8890)
{
  buffer_ = std::vector<unsigned char>(1024);

  // Listen for state packets from the drone
  thread_ = std::thread(
    [this]()
    {
      for (;;) {
        size_t r = socket_.receive(asio::buffer(buffer_));
        process_packet(r);
      }
    });
}

// Process a state packet from the drone
void StateSocket::process_packet(size_t r)
{
  last_time_ = driver_->now();

  if (!receiving_) {
    bool v2 = (r > 0) && buffer_[0] == 'm';
    RCLCPP_INFO(driver_->get_logger(), "Receiving state, SDK %s", (v2 ? "v2.0" : "v1.3"));
    receiving_ = true;
  }

  // Unpack and publish
  // TODO
}

} // namespace tello_driver