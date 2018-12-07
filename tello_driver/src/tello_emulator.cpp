#include <cstdlib>
#include <iostream>
#include <thread>

#include <asio.hpp>

using asio::ip::udp;

//=====================================================================================
// Tello emulator
// Implements Tello SDK v1.3
//=====================================================================================

void emulator()
{
  asio::io_service io_service;

  std::array<char, 1024>buffer;

  udp::socket command_socket(io_service, udp::endpoint(udp::v4(), 8889));

  udp::endpoint state_remote_endpoint{udp::v4(), 8890};
  udp::endpoint video_remote_endpoint{udp::v4(), 11111};

  udp::socket state_socket{io_service, udp::endpoint(udp::v4(), 0)};
  udp::socket video_socket{io_service, udp::endpoint(udp::v4(), 0)};

  std::thread state_thread;
  std::thread video_thread;

  bool connected = false;
  bool streaming = false;

  for (;;)
  {
    // Wait for a message from the controller
    udp::endpoint sender_endpoint;
    size_t length = command_socket.receive_from(asio::buffer(buffer), sender_endpoint);

    std::string address = sender_endpoint.address().to_string();
    unsigned short port = sender_endpoint.port();

    std::string command(std::begin(buffer), std::begin(buffer) + length);
    std::cout << "Drone heard '" << command << "' from " << address << ":" << port << std::endl;

    // Respond with an "ok"
    command_socket.send_to(asio::buffer(std::string("ok")), sender_endpoint);

    // If we heard "command" then start sending state messages
    if (!connected && command == "command")
    {
      connected = true;

      state_thread = std::thread(
        [&state_socket, &state_remote_endpoint]()
        {
          for (;;)
          {
            state_socket.send_to(asio::buffer(std::string("some state")), state_remote_endpoint);
            sleep(1);
          }
        });
    }

    // If we heard "streamon" then start sending video messages
    if (!streaming && command == "streamon")
    {
      streaming = true;

      video_thread = std::thread(
        [&video_socket, &video_remote_endpoint]()
        {
          for (;;)
          {
            video_socket.send_to(asio::buffer(std::string("some video")), video_remote_endpoint);
            sleep(1);
          }
        });
    }
  }
}

int main(int argc, char* argv[])
{
  try
  {
    emulator();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}