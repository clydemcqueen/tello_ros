#include <iostream>
#include <asio.hpp>

#include "rclcpp/rclcpp.hpp"

using asio::ip::udp;

namespace tello_driver {

// Message publish rate in Hz
constexpr int SPIN_RATE = 100;

//=====================================================================================
// An abstract class that receives data on a socket.
// Override _handler to do something with the data.
//=====================================================================================

class Receiver
{
public:

  Receiver(udp::socket &socket) : _socket(socket)
  {
    std::string address = socket.local_endpoint().address().to_string();
    unsigned short port = socket.local_endpoint().port();
    std::cout << "Listening on " << address << ":" << port << std::endl;

    // Start receiving
    do_receive();
  }

  ~Receiver()
  {
    // Stop receiving
    _socket.cancel();
  }

  // Have we received anything yet?
  bool is_active()
  {
    return _active;
  }

private:

  void do_receive()
  {
    udp::endpoint sender_endpoint;

    // Async call returns immediately
    _socket.async_receive_from(asio::buffer(_data, _max_length), sender_endpoint,
      [this](std::error_code ec, std::size_t bytes_recvd)
      {
        if (!ec && bytes_recvd > 0)
        {
          _active = true;
          _handler(ec, bytes_recvd);
        }

        // Receive again
        do_receive();
      });
  }

protected:

  virtual void _handler(std::error_code ec, std::size_t bytes_recvd) = 0;

  udp::socket &_socket;
  bool _active = false;
  static const size_t _max_length = 1024;
  char _data[_max_length];
};

//=====================================================================================
// Tello driver class.
// Implements the Tello SDK v1.3.
//=====================================================================================

class TelloDriver : public rclcpp::Node
{
  class CommandReceiver : public Receiver
  {
    // Responses to commands, often just 'ok' or 'error'
    void _handler(std::error_code ec, std::size_t bytes_recvd) override
    {
      std::cout << "Command response" << std::endl;
    }

  public:

    CommandReceiver(udp::socket &socket) : Receiver(socket) {}
  };

  class StateReceiver : public Receiver
  {
    // Drone state information
    void _handler(std::error_code ec, std::size_t bytes_recvd) override
    {
      std::cout << "State response" << std::endl;
    }

  public:

    StateReceiver(udp::socket &socket) : Receiver(socket) {}
  };

  class VideoReceiver : public Receiver
  {
    // H264-encoded video packets
    void _handler(std::error_code ec, std::size_t bytes_recvd) override
    {
      std::cout << "Video response" << std::endl;

      // TODO get frame(s) from h264
      // TODO for each frame, convert to ROS image
      // TODO publish
    }

  public:

    VideoReceiver(udp::socket &socket) : Receiver(socket) {}
  };

public:

  explicit TelloDriver(asio::io_service &io_service) : Node("tello_driver"),
    _command_remote_endpoint(udp::v4() /*_drone_ip*/, 8889),
    _command_socket(io_service, udp::endpoint(udp::v4(), 0)),
    _state_socket(io_service, udp::endpoint(udp::v4(), 8890)),
    _video_socket(io_service, udp::endpoint(udp::v4(), 11111)),
    _command_receiver(_command_socket),
    _state_receiver(_state_socket),
    _video_receiver(_video_socket)
  {
    // TODO set up publishers

    // TODO set up subscribers
  }

  ~TelloDriver()
  {
    // TODO not sure about the sequence -- will stuff get closed correctly?
  };

  void spin_once()
  {
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
  }

private:

  // Do work every 1s
  void spin_1s()
  {
    if (!_command_receiver.is_active())
    {
      std::cout << "Initialize SDK" << std::endl;
      _command_socket.send_to(asio::buffer(std::string("command")), _command_remote_endpoint);
    }

    if (_command_receiver.is_active() && !_video_receiver.is_active())
    {
      std::cout << "Stream video" << std::endl;
      _command_socket.send_to(asio::buffer(std::string("streamon")), _command_remote_endpoint);
    }
  }

  // Do work every 5s
  void spin_5s()
  {
    if (_command_receiver.is_active())
    {
      // Drone will land if there's no communication for 15s
      std::cout << "Keep flying" << std::endl;
      _command_socket.send_to(asio::buffer(std::string("command")), _command_remote_endpoint);
    }
  }

  // Drone IP address
  asio::ip::address_v4 _drone_ip = asio::ip::address_v4::from_string("192.168.10.1");

  // Drone command endpoint
  udp::endpoint _command_remote_endpoint;

  // Sockets
  udp::socket _command_socket;
  udp::socket _state_socket;
  udp::socket _video_socket;

  // Receivers
  CommandReceiver _command_receiver;
  StateReceiver _state_receiver;
  VideoReceiver _video_receiver;
};

} // namespace tello_driver


int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);
  rclcpp::Rate r(tello_driver::SPIN_RATE);

  // Spin the socket service
  asio::io_service io_service;
  std::cout << "Spinning io_service" << std::endl;
  asio::io_service::work work(io_service);
  std::thread thread1([&io_service](){ io_service.run(); });

  // Spin the ROS node
  auto node = std::make_shared<tello_driver::TelloDriver>(io_service);
  std::cout << "Spinning TelloDriver" << std::endl;
  while (rclcpp::ok())
  {
    // Do our work
    node->spin_once();

    // Respond to incoming ROS messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  // Join thread(s)
  io_service.stop();  // Stop service
  thread1.join();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
