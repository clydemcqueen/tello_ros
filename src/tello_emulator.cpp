#include <cstdlib>
#include <iostream>

#include <asio.hpp>

using asio::ip::udp;

void emulator(asio::io_service& io_service)
{
  const size_t max_length = 1024;
  udp::socket sock(io_service, udp::endpoint(udp::v4(), 8889));
  for (;;)
  {
    char data[max_length];
    udp::endpoint sender_endpoint;
    size_t length = sock.receive_from(asio::buffer(data, max_length), sender_endpoint);

    std::string address = sender_endpoint.address().to_string();
    unsigned short port = sender_endpoint.port();

    std::string str(data, length);
    std::cout << "Drone heard '" << str << "' from " << address << ":" << port << std::endl;

    sock.send_to(asio::buffer(std::string("ok")), sender_endpoint);

    sock.send_to(asio::buffer(std::string("foo")), udp::endpoint(udp::v4(), 8890));
    sock.send_to(asio::buffer(std::string("foo")), udp::endpoint(udp::v4(), 11111));
  }
}

int main(int argc, char* argv[])
{
  try
  {
    asio::io_service io_service;
    emulator(io_service);
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}