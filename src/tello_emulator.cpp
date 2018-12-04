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

  const size_t max_length = 1024;
  char data[max_length];

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
    size_t length = command_socket.receive_from(asio::buffer(data, max_length), sender_endpoint);
    
    std::string address = sender_endpoint.address().to_string();
    unsigned short port = sender_endpoint.port();

    std::string str(data, length);
    std::cout << "Drone heard '" << str << "' from " << address << ":" << port << std::endl;

    // Respond with an "ok"
    command_socket.send_to(asio::buffer(std::string("ok")), sender_endpoint);

    // If we heard "command", then start sending state messages
    if (!connected && std::string(data, length) == "command")
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
    if (!streaming && std::string(data, length) == "streamon")
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