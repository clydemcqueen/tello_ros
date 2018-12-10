#include <asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/flip.hpp"

#include "h264decoder.hpp"

using asio::ip::udp;

namespace tello_driver {

class CommandSocket;
class StateSocket;
class VideoSocket;

enum class SDK
{
  unknown,
  v1_3,
  v2_0
};

//=====================================================================================
// Tello driver implements Tello SDK 1.3 and 2.0
//=====================================================================================

class TelloDriver : public rclcpp::Node
{
public:

  explicit TelloDriver();

  ~TelloDriver();

  void spin_once();

  void lock() { mtx_.lock(); }
  void unlock() { mtx_.unlock(); }

  bool connected();

  SDK sdk() { return sdk_; }
  void set_sdk(SDK sdk);

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<tello_msgs::msg::FlightData>::SharedPtr flight_data_pub_;

private:

  void spin_1s();
  void spin_5s();

  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  void takeoff_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void land_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void flip_callback(const tello_msgs::msg::Flip::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Sockets
  std::unique_ptr<CommandSocket> command_socket_;
  std::unique_ptr<StateSocket> state_socket_;
  std::unique_ptr<VideoSocket> video_socket_;

  // Mutex, protects this object and std::cout
  std::mutex mtx_;

  // ROS subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_sub_;
  rclcpp::Subscription<tello_msgs::msg::Flip>::SharedPtr flip_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // State
  SDK sdk_ = SDK::unknown;
};

//=====================================================================================
// Abstract socket
//=====================================================================================

class TelloSocket
{
public:

  TelloSocket(TelloDriver *driver, unsigned short port) : driver_(driver),
    socket_(io_service_, udp::endpoint(udp::v4(), port)) {}

  bool receiving() { return receiving_; }
  void reset() { receiving_ = false; }
  rclcpp::Time last_time() { return last_time_; }

protected:

  virtual void process_packet(size_t r) = 0;

  TelloDriver *driver_;                 // Pointer to the driver node
  asio::io_service io_service_;         // Manages io for this socket
  udp::socket socket_;                  // The socket
  std::thread thread_;                  // Each socket received on it's own thread

  bool receiving_ = false;              // Have we received a packet on this socket?
  rclcpp::Time last_time_;              // Time of most recent packet
  std::vector<unsigned char> buffer_;   // Packet buffer
};

//=====================================================================================
// Command socket
//=====================================================================================

class CommandSocket : public TelloSocket
{
public:

  CommandSocket(TelloDriver *driver);

  void send_command(std::string command);

private:

  void process_packet(size_t r) override;

#undef LOCAL_EMULATION
#ifdef LOCAL_EMULATION
  udp::endpoint remote_endpoint_{udp::v4(), 8889};
#else
  udp::endpoint remote_endpoint_{asio::ip::address_v4::from_string("192.168.10.1"), 8889};
#endif
  bool local_endpoint_bound_ = false;
};

//=====================================================================================
// State socket
//=====================================================================================

class StateSocket : public TelloSocket
{
public:

  StateSocket(TelloDriver *driver);

private:

  void process_packet(size_t r) override;
};

//=====================================================================================
// Video socket
//=====================================================================================

class VideoSocket : public TelloSocket
{
public:

  VideoSocket(TelloDriver *driver);

private:

  // Process a video packet from the drone
  void process_packet(size_t r) override;

  // Decode frames
  void decode_frames();

  std::vector<unsigned char> seq_buffer_;
  size_t seq_buffer_next_ = 0;
  int seq_buffer_num_packets_ = 0;

  H264Decoder decoder_;
  ConverterRGB24 converter_;
};

} // namespace tello_driver
