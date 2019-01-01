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

# Emulate colcon
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Asio requires threading
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)

# Local includes
include_directories(
  include
  h264decoder
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS})

# h264decoder uses deprecated ffmpeg calls, turn off the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

#=============
# Tello driver node
#=============

add_executable(tello_driver
  src/tello_driver.cpp
  src/tello_socket.cpp
  src/command_socket.cpp
  src/state_socket.cpp
  src/video_socket.cpp
  h264decoder/h264decoder.cpp)

ament_target_dependencies(tello_driver
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  tello_msgs)

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
# Tello joy node
#=============

add_executable(tello_joy src/tello_joy.cpp)

ament_target_dependencies(tello_joy
  geometry_msgs
  rclcpp
  sensor_msgs
  tello_msgs)

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  tello_driver
  tello_emulator
  tello_joy
  DESTINATION lib/tello_driver
)

# Install launch directory
install(
  DIRECTORY
  launch
  DESTINATION share/tello_driver
)

#=============
# Run ament macros
#=============

ament_package()
