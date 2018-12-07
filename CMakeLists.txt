cmake_minimum_required(VERSION 3.5)
project(tello_driver)

#=============
# Setup
#=============

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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Local includes
include_directories(h264decoder)

# h264decoder uses deprecated ffmpeg calls, turn off the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

#=============
# Tello driver node
#=============

add_executable(tello_driver src/tello_driver.cpp h264decoder/h264decoder.cpp)

ament_target_dependencies(tello_driver cv_bridge OpenCV rclcpp sensor_msgs)

# Can't find_package(ffmpeg), so ament_target_dependencies won't work
target_link_libraries(tello_driver avcodec avutil swscale)

# Tell Asio to use std::, not boost::
target_compile_definitions(tello_driver
  PRIVATE ASIO_STANDALONE
  PRIVATE ASIO_HAS_STD_CHRONO)

#=============
# Tello emulator
#=============

add_executable(tello_emulator src/tello_emulator.cpp)

ament_target_dependencies(tello_emulator)

target_compile_definitions(tello_emulator
  PRIVATE ASIO_STANDALONE
  PRIVATE ASIO_HAS_STD_CHRONO)

#=============
# Run ament macros
#=============

ament_package()
