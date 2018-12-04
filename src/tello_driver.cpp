#include <iostream>
#include <asio.hpp>

#include "rclcpp/rclcpp.hpp"

using asio::ip::udp;

namespace tello_driver {

// Message publish rate in Hz
constexpr int SPIN_RATE = 100;

//=====================================================================================
// Tello driver
// Implements Tello SDK v1.3
//=====================================================================================

class TelloDriver : public rclcpp::Node
{
public:

  explicit TelloDriver() : Node("tello_driver")
  {
    // TODO set up publishers
    // TODO set up subscribers

    // Listen for state packets from the drone
    state_thread_ = std::thread(
      [this]()
      {
        for (;;)
        {
          size_t r = state_socket_.receive(asio::buffer(state_buffer_, max_length_));
          mtx_.lock();
          process_state(r);
          mtx_.unlock();
        }
      });

    // Listen for video packets from the drone
    video_thread_ = std::thread(
      [this]()
      {
        for (;;)
        {
          size_t r = video_socket_.receive(asio::buffer(video_buffer_, max_length_));
          mtx_.lock();
          process_video(r);
          mtx_.unlock();
        }
      });
  }

  ~TelloDriver()
  {
  };

  void spin_once()
  {
    mtx_.lock();

    static unsigned int counter = 0;
    counter++;

    if (counter % SPIN_RATE == 0)
    {
      spin_1s();
    }

    if (counter % (5 * SPIN_RATE) == 0)
    {
      spin_5s();
    }

    mtx_.unlock();
  }

private:

  // Do work every 1s
  void spin_1s()
  {
    if (connected_ && now() - last_state_time_ > rclcpp::Duration(5, 0))
    {
      std::cout << "No state received for 5s" << std::endl;
      connected_ = false;
    }

    if (streaming_ && now() - last_video_time_ > rclcpp::Duration(5, 0))
    {
      std::cout << "No video received for 5s" << std::endl;
      streaming_ = false;
    }

    if (!connected_)
    {
      // Activate the SDK, and start sending state packets
      std::cout << "Activating SDK" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("command")), command_remote_endpoint_);
    }

    if (connected_ && !streaming_)
    {
      // Start video
      std::cout << "Activating video" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("streamon")), command_remote_endpoint_);
    }
  }

  // Do work every 5s
  void spin_5s()
  {
    if (connected_ && streaming_)
    {
      // The drone will auto-land if it hears nothing for 15s
      std::cout << "Keep alive" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("command")), command_remote_endpoint_);
    }
  }

  // Process a state packet from the drone
  void process_state(size_t r)
  {
    last_state_time_ = now();

    if (!connected_)
    {
      std::cout << "Receiving state! " << r << std::endl;
      connected_ = true;
    }
  }

  // Process a video packet from the drone
  void process_video(size_t r)
  {
    last_video_time_ = now();

    if (!streaming_)
    {
      std::cout << "Receiving video! " << r << std::endl;
      streaming_ = true;
    }
  }

  // Asio setup
  asio::io_service io_service_;
  asio::ip::address_v4 drone_ip_ = asio::ip::address_v4::from_string("192.168.10.1");
  udp::endpoint command_local_endpoint_{udp::v4(), 0};
#undef LOCAL_EMULATION
#ifdef LOCAL_EMULATION
  udp::endpoint command_remote_endpoint_{udp::v4(), 8889};
#else
  udp::endpoint command_remote_endpoint_{drone_ip_, 8889};
#endif
  udp::endpoint state_local_endpoint_{udp::v4(), 8890};
  udp::endpoint video_local_endpoint_{udp::v4(), 11111};
  udp::socket command_socket_{io_service_, command_local_endpoint_};
  udp::socket state_socket_{io_service_, state_local_endpoint_};
  udp::socket video_socket_{io_service_, video_local_endpoint_};

  // Buffers
  static const size_t max_length_ = 1024;
  char state_buffer_[max_length_];
  char video_buffer_[max_length_];

  // Threads
  std::mutex mtx_;  // Used to protect all members, std::cout, etc.
  std::thread state_thread_;
  std::thread video_thread_;

  // State
  bool connected_ = false;
  bool streaming_ = false;
  rclcpp::Time last_state_time_;
  rclcpp::Time last_video_time_;
};

} // namespace tello_driver


int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::Rate r(tello_driver::SPIN_RATE);
  auto node = std::make_shared<tello_driver::TelloDriver>();

  while (rclcpp::ok())
  {
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
