cmake_minimum_required(VERSION 3.5)
project(tello_gazebo)

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
  message("Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)  # Note uppercase variables in /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
)

#=============
# Tello Gazebo plugin
#=============

add_library(
  TelloPlugin SHARED
  src/tello_plugin.cpp
)

ament_target_dependencies(
  TelloPlugin
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
  std_msgs
  tello_msgs
)

#=============
# Install
#=============

# Install targets
install(
  TARGETS TelloPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install world and launch files
install(
  DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

## Install Python scripts
install(
  PROGRAMS src/inject_entity.py
  DESTINATION lib/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
