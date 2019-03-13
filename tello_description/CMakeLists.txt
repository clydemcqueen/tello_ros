cmake_minimum_required(VERSION 3.5)
project(tello_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/tello_description
)

ament_package()
