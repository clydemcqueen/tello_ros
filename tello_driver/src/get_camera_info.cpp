#include <iostream>
#include <fstream>

#include "sensor_msgs/msg/camera_info.hpp"

#ifdef RUN_INSIDE_CLION
std::string cfg_path("../cfg/camera_info.txt");
#else
std::string cfg_path("install/tello_driver/share/tello_driver/cfg/camera_info.txt");
#endif

bool get_camera_info(sensor_msgs::msg::CameraInfo &info)
{
  // File format: 2 ints and 9 floats, separated by whitespace:
  // height width fx fy cx cy k1 k2 t1 t2 t3

  std::ifstream file;
  file.open(cfg_path);
  if (!file) {
    return false;
  }

  uint32_t height, width;
  double fx, fy, cx, cy, k1, k2, t1, t2, t3;
  file >> height >> width;
  file >> fx >> fy;
  file >> cx >> cy;
  file >> k1 >> k2 >> t1 >> t2 >> t3;
  file.close();

  // See https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

  info.header.frame_id = "camera_frame";
  info.height = height;
  info.width = width;
  info.distortion_model = "plumb_bob";

  info.d.push_back(k1);
  info.d.push_back(k2);
  info.d.push_back(t1);
  info.d.push_back(t2);
  info.d.push_back(t3);

  info.k[0] = fx;
  info.k[1] = 0;
  info.k[2] = cx;
  info.k[3] = 0;
  info.k[4] = fy;
  info.k[5] = cy;
  info.k[6] = 0;
  info.k[7] = 0;
  info.k[8] = 1;

  info.p[0] = fx;
  info.p[1] = 0;
  info.p[2] = cx;
  info.p[3] = 0;
  info.p[4] = 0;
  info.p[5] = fy;
  info.p[6] = cy;
  info.p[7] = 0;
  info.p[8] = 0;
  info.p[9] = 0;
  info.p[10] = 1;
  info.p[11] = 0;

  return true;
}

