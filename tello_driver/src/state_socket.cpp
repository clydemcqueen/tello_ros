#include "tello_driver.hpp"

namespace tello_driver {

// v1.3: pitch:0;roll:-1;yaw:0;vgx:0;vgy:0;vgz:0;templ:46;temph:49;tof:10;h:0;bat:100;baro:18.92;time:0;agx:-12.00;agy:16.00;agz:-993.00;
// v2.0: mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;roll:5;yaw:0;vgx:0;vgy:0;vgz:0;templ:44;temph:47;tof:10;h:0;bat:100;baro:17.84;time:0;agx:10.00;agy:-98.00;agz:-988.00;

StateSocket::StateSocket(TelloDriver *driver) : TelloSocket(driver, 8890)
{
  buffer_ = std::vector<unsigned char>(1024);
  listen();
}

// Process a state packet from the drone
void StateSocket::process_packet(size_t r)
{
  std::lock_guard<std::mutex> lock(mtx_);

  static std::map<SDK, std::string> enum_names{
    {SDK::unknown, "unknown"},
    {SDK::v1_3, "v1.3"},
    {SDK::v2_0, "v2.0"}};

  receive_time_ = driver_->now();

  if (!receiving_) {
    sdk_ = (r > 0) && buffer_[0] == 'm' ? SDK::v2_0 : SDK::v1_3;
    RCLCPP_INFO(driver_->get_logger(), "Receiving state, SDK version %s", enum_names[sdk_].c_str());
    receiving_ = true;
  }

  // Unpack and publish
  if (driver_->count_subscribers(driver_->flight_data_pub_->get_topic_name()) > 0) {
    tello_msgs::msg::FlightData msg;
    // TODO
    driver_->flight_data_pub_->publish(msg);
  }
}

} // namespace tello_driver