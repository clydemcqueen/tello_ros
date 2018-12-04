cmake_minimum_required(VERSION 3.5)
project(tello_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Asio requires threading
set(CMAKE_CXX_FLAGS "-pthread")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Local includes
include_directories(include)

# Tello driver node
add_executable(tello_driver src/tello_driver.cpp)

ament_target_dependencies(tello_driver rclcpp)

# Tell Asio to use std::, not boost::
target_compile_definitions(tello_driver
  PRIVATE ASIO_ENABLE_HANDLER_TRACKING
  PRIVATE ASIO_STANDALONE
  PRIVATE ASIO_HAS_STD_CHRONO)

# Tello emulator
add_executable(tello_emulator src/tello_emulator.cpp)

ament_target_dependencies(tello_emulator)

target_compile_definitions(tello_emulator
  PRIVATE ASIO_ENABLE_HANDLER_TRACKING
  PRIVATE ASIO_STANDALONE
  PRIVATE ASIO_HAS_STD_CHRONO)

ament_package()
