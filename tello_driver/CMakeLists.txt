cmake_minimum_required(VERSION 3.5)
project(tello_driver)

#=============
# Setup
#=============

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon
if ($ENV{CLION_IDE})
  message(STATUS "Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# Asio requires threading
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  h264decoder
  ${ros2_shared_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS})

# h264decoder uses deprecated ffmpeg calls, turn off the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# Tello driver node
#=============

add_library(tello_driver_node SHARED
  src/tello_driver_node.cpp
  src/tello_socket.cpp
  src/command_socket.cpp
  src/state_socket.cpp
  src/video_socket.cpp
  h264decoder/h264decoder.cpp)

target_compile_definitions(tello_driver_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(tello_driver_node
  camera_calibration_parsers
  class_loader
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  ros2_shared
  sensor_msgs
  std_msgs
  tello_msgs)

rclcpp_components_register_nodes(tello_driver_node "tello_driver::TelloDriverNode")
set(node_plugins "${node_plugins}tello_driver::TelloDriverNode;$<TARGET_FILE:tello_driver_node>\n")

# Can't find_package(ffmpeg), so ament_target_dependencies won't work
target_link_libraries(tello_driver_node avcodec avutil swscale)

# Tell Asio to use std::, not boost::
target_compile_definitions(tello_driver_node
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

add_library(tello_joy_node SHARED
  src/tello_joy_node.cpp)

target_compile_definitions(tello_joy_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(tello_joy_node
  class_loader
  geometry_msgs
  rclcpp
  sensor_msgs
  tello_msgs)

rclcpp_components_register_nodes(tello_joy_node "tello_joy::TelloJoyNode")
set(node_plugins "${node_plugins}tello_joy::TelloJoyNode;$<TARGET_FILE:tello_joy_node>\n")

#=============
# Export incantations, see https://github.com/ros2/demos/blob/master/composition/CMakeLists.txt
#=============

if (NOT WIN32)
  ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif ()

#=============
# Tello driver main
#=============

add_executable(tello_driver_main
  src/tello_driver_main.cpp)

target_link_libraries(tello_driver_main
  tello_driver_node)

ament_target_dependencies(tello_driver_main
  rclcpp)

#=============
# Tello joy and driver main
#=============

add_executable(tello_joy_and_driver_main
  src/tello_joy_main.cpp)

target_link_libraries(tello_joy_and_driver_main
  tello_joy_node
  tello_driver_node)

ament_target_dependencies(tello_joy_and_driver_main
  rclcpp)

#=============
# Install
#=============

# Install nodes
install(
  TARGETS tello_driver_node tello_joy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS tello_driver_main tello_joy_and_driver_main tello_emulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY
  cfg
  launch
  DESTINATION share/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
