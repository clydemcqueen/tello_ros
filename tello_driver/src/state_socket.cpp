#include "tello_driver_node.hpp"

#include <regex>

namespace tello_driver
{

  // Goals:
  // * make the data useful by parsing all documented fields
  // * some future SDK version might introduce new field types, so don't parse undocumented fields
  // * send the raw string as well

  StateSocket::StateSocket(TelloDriverNode *driver, unsigned short data_port) : TelloSocket(driver, data_port)
  {
    buffer_ = std::vector<unsigned char>(1024);
    listen();
  }

// Process a state packet from the drone, runs at 10Hz
  void StateSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    static std::map<uint8_t, std::string> enum_names{
      {tello_msgs::msg::FlightData::SDK_UNKNOWN, "unknown"},
      {tello_msgs::msg::FlightData::SDK_1_3,     "v1.3"},
      {tello_msgs::msg::FlightData::SDK_2_0,     "v2.0"}};

    receive_time_ = driver_->now();

    if (receiving_ && driver_->count_subscribers(driver_->flight_data_pub_->get_topic_name()) == 0) {
      // Nothing to do
      return;
    }

    // Split on ";" and ":" and generate a key:value map
    std::map<std::string, std::string> fields;
    std::string raw(buffer_.begin(), buffer_.begin() + r);
    std::regex re("([^:]+):([^;]+);");
    for (auto i = std::sregex_iterator(raw.begin(), raw.end(), re); i != std::sregex_iterator(); ++i) {
      auto match = *i;
      fields[match[1]] = match[2];
    }

    // First message?
    if (!receiving_) {
      receiving_ = true;

      // Hack to figure out the SDK version
      sdk_ = tello_msgs::msg::FlightData::SDK_1_3;
      auto i = fields.find("mid");
      if (i != fields.end() && i->second != "257") {
        sdk_ = tello_msgs::msg::FlightData::SDK_2_0;
      }
      RCLCPP_INFO(driver_->get_logger(), "Receiving state, SDK version %s", enum_names[sdk_].c_str());
    }

    // Only send ROS messages if there are subscribers
    if (driver_->count_subscribers(driver_->flight_data_pub_->get_topic_name()) > 0) {
      tello_msgs::msg::FlightData msg;
      msg.header.stamp = receive_time_;
      msg.raw = raw;
      msg.sdk = sdk_;

      try {

        if (sdk_ == tello_msgs::msg::FlightData::SDK_2_0) {
          msg.mid = std::stoi(fields["mid"]);
          msg.x = std::stoi(fields["x"]);
          msg.y = std::stoi(fields["y"]);
          msg.z = std::stoi(fields["z"]);
        }

        msg.pitch = std::stoi(fields["pitch"]);
        msg.roll = std::stoi(fields["roll"]);
        msg.yaw = std::stoi(fields["yaw"]);
        msg.vgx = std::stoi(fields["vgx"]);
        msg.vgy = std::stoi(fields["vgy"]);
        msg.vgz = std::stoi(fields["vgz"]);
        msg.templ = std::stoi(fields["templ"]);
        msg.temph = std::stoi(fields["temph"]);
        msg.tof = std::stoi(fields["tof"]);
        msg.h = std::stoi(fields["h"]);
        msg.bat = std::stoi(fields["bat"]);
        msg.baro = std::stof(fields["baro"]);
        msg.time = std::stoi(fields["time"]);
        msg.agx = std::stof(fields["agx"]);
        msg.agy = std::stof(fields["agy"]);
        msg.agz = std::stof(fields["agz"]);

      } catch (std::exception e) {
        RCLCPP_ERROR(driver_->get_logger(), "Can't parse flight data");
        return;
      }

      driver_->flight_data_pub_->publish(msg);
    }
  }

} // namespace tello_driver