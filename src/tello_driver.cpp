#include <iostream>
#include <asio.hpp>
#include <libavutil/frame.h>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

#include "h264decoder.hpp"

using asio::ip::udp;

// Notes on Tello video:
// -- frames are always 960x720.
// -- frames are split into UDP packets of length 1460.
// -- normal frames are ~10k, or about 8 UDP packets.
// -- keyframes are ~35k, or about 25 UDP packets.
// -- keyframes are always preceded by an 8-byte UDP packet and a 13-byte UDP packet -- markers?
// -- the h264 parser will consume the 8-byte packet, the 13-byte packet and the entire keyframe without
//    generating a frame. Presumably the keyframe is stored in the parser and referenced later.

namespace tello_driver {

// Message publish rate in Hz
constexpr int SPIN_RATE = 100;

//=====================================================================================
// Tello driver
// Implements Tello SDK v1.3
//=====================================================================================

class TelloDriver : public rclcpp::Node
{
public:

  explicit TelloDriver() : Node("tello_driver")
  {
    // Publisher(s)
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    // Listen for state packets from the drone
    state_thread_ = std::thread(
      [this]()
      {
        for (;;)
        {
          size_t r = state_socket_.receive(asio::buffer(state_buffer_));
          mtx_.lock();
          process_state(r);
          mtx_.unlock();
        }
      });

    // Listen for video packets from the drone
    video_thread_ = std::thread(
      [this]()
      {
        for (;;)
        {
          size_t r = video_socket_.receive(asio::buffer(video_buffer_));
          mtx_.lock();
          process_video(r);
          mtx_.unlock();
        }
      });
  }

  ~TelloDriver()
  {
  };

  void spin_once()
  {
    mtx_.lock();

    static unsigned int counter = 0;
    counter++;

    if (counter % SPIN_RATE == 0)
    {
      spin_1s();
    }

    if (counter % (5 * SPIN_RATE) == 0)
    {
      spin_5s();
    }

    mtx_.unlock();
  }

private:

  // Do work every 1s
  void spin_1s()
  {
    if (connected_ && now() - last_state_time_ > rclcpp::Duration(5, 0))
    {
      std::cout << "No state received for 5s" << std::endl;
      connected_ = false;
    }

    if (streaming_ && now() - last_video_time_ > rclcpp::Duration(5, 0))
    {
      std::cout << "No video received for 5s" << std::endl;
      streaming_ = false;
    }

    if (!connected_)
    {
      // Activate the SDK, and start sending state packets
      std::cout << "Activating SDK" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("command")), command_remote_endpoint_);
    }

    if (connected_ && !streaming_)
    {
      // Start video
      std::cout << "Activating video" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("streamon")), command_remote_endpoint_);
    }
  }

  // Do work every 5s
  void spin_5s()
  {
    if (connected_ && streaming_)
    {
      // The drone will auto-land if it hears nothing for 15s
      std::cout << "Keep alive" << std::endl;
      command_socket_.send_to(asio::buffer(std::string("command")), command_remote_endpoint_);
    }
  }

  // Process a state packet from the drone
  void process_state(size_t r)
  {
    last_state_time_ = now();

    if (!connected_)
    {
      std::cout << "Receiving state! " << r << std::endl;
      connected_ = true;
    }
  }

  // Process a video packet from the drone
  void process_video(size_t r)
  {
    last_video_time_ = now();

    if (!streaming_)
    {
      // First packet
      std::cout << "Receiving video! " << r << std::endl;
      streaming_ = true;
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }

    if (seq_buffer_next_ + r >= seq_buffer_.size())
    {
      std::cout << "ERROR! Video buffer overflow, dropping sequence" << std::endl;
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
      return;
    }

    std::copy(video_buffer_.begin(), video_buffer_.begin() + r, seq_buffer_.begin() + seq_buffer_next_);
    seq_buffer_next_ += r;
    seq_buffer_num_packets_++;

    // If the packet is < 1460 bytes then it's the last packet in the sequence
    if (r < 1460)
    {
      decode_frames();

      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }
  }

  // Decode frames
  void decode_frames()
  {
    size_t next = 0;

    try
    {
      while (next < seq_buffer_next_)
      {
        // Parse h264
        ssize_t consumed = decoder_.parse((unsigned char*)seq_buffer_.begin() + next, seq_buffer_next_ - next);

        // Is a frame available?
        if (decoder_.is_frame_available())
        {
          // Decode the frame
          const AVFrame& frame = decoder_.decode_frame();

          // Convert pixels from YUV420P to BGR24
          int size = converter_.predict_size(frame.width, frame.height);
          unsigned char bgr24[size];
          converter_.convert(frame, bgr24);

          // Convert to cv::Mat
          cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24};

          // Display
          // TODO make this conditional
          cv::imshow("frame", mat);
          cv::waitKey(1);

          // Publish a ROS message
          // TODO only publish if somebody is subscribing
          std_msgs::msg::Header header{};
          header.frame_id = "camera_frame";
          header.stamp = now();
          cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, mat};
          image_pub_->publish(image.toImageMsg());
        }

        next += consumed;
      }
    }
    catch (std::runtime_error e)
    {
      std::cout << e.what() << std::endl;
    }
  }

  // Asio setup
  asio::io_service io_service_;
  asio::ip::address_v4 drone_ip_ = asio::ip::address_v4::from_string("192.168.10.1");
  udp::endpoint command_local_endpoint_{udp::v4(), 0};
#undef LOCAL_EMULATION
#ifdef LOCAL_EMULATION
  udp::endpoint command_remote_endpoint_{udp::v4(), 8889};
#else
  udp::endpoint command_remote_endpoint_{drone_ip_, 8889};
#endif
  udp::endpoint state_local_endpoint_{udp::v4(), 8890};
  udp::endpoint video_local_endpoint_{udp::v4(), 11111};
  udp::socket command_socket_{io_service_, command_local_endpoint_};
  udp::socket state_socket_{io_service_, state_local_endpoint_};
  udp::socket video_socket_{io_service_, video_local_endpoint_};

  // State buffer holds 1 state packet
  std::array<char, 1024>state_buffer_;

  // Video buffer holds 1 video packet
  std::array<unsigned char, 1024*2>video_buffer_;

  // Sequence buffer holds N video packets
  std::array<unsigned char, 1024*64>seq_buffer_;
  size_t  seq_buffer_next_ = 0;
  int seq_buffer_num_packets_ = 0;

  // Threads
  std::mutex mtx_;  // Used to protect all members, std::cout, etc.
  std::thread state_thread_;
  std::thread video_thread_;

  // State
  bool connected_ = false;
  bool streaming_ = false;
  rclcpp::Time last_state_time_;
  rclcpp::Time last_video_time_;

  // Decoder
  H264Decoder decoder_;
  ConverterRGB24 converter_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

} // namespace tello_driver


int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::Rate r(tello_driver::SPIN_RATE);
  auto node = std::make_shared<tello_driver::TelloDriver>();

  while (rclcpp::ok())
  {
    // Do our work
    node->spin_once();

    // Respond to incoming ROS messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
