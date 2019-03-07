#include <chrono>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tello_msgs/srv/tello_action.hpp"

using namespace std::chrono_literals;

namespace tello_gazebo {

class TelloPlugin : public gazebo::ModelPlugin
{
  //gazebo::physics::Model model_;
  gazebo::physics::LinkPtr base_link_;

  // Force will be applied to the center_of_mass_ (body frame)
  // TODO is it possible to get this from the link?
  ignition::math::Vector3d center_of_mass_ {0, 0, 0};

  // Target velocity
  geometry_msgs::msg::Twist twist_;

  // Connection to Gazebo message bus
  gazebo::event::ConnectionPtr update_connection_;

  // GazeboROS node
  gazebo_ros::Node::SharedPtr node_;

  // ROS publishers
  rclcpp::Publisher<tello_msgs::msg::FlightData>::SharedPtr flight_data_pub_;
  rclcpp::Publisher<tello_msgs::msg::TelloResponse >::SharedPtr tello_response_pub_;

  // ROS services
  rclcpp::Service<tello_msgs::srv::TelloAction>::SharedPtr command_srv_;

  // ROS subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr timer_;

public:

  TelloPlugin()
  {
  }

  ~TelloPlugin()
  {
  }

  // Called once when the plugin is loaded.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    std::string ns;
    std::string link_name {"base_link"};

    // In theory we can move much of this config into the <ros> tag, but this appears unfinished in Crystal
    if (sdf->HasElement("ns")) {
      ns = sdf->GetElement("ns")->Get<std::string>();
    }
    if (sdf->HasElement("link_name")) {
      link_name = sdf->GetElement("link_name")->Get<std::string>();
    }
    if (sdf->HasElement("center_of_mass")) {
      center_of_mass_ = sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
    }

    std::cout << std::endl;
    std::cout << "TELLO PLUGIN" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "ns: " << ns << std::endl;
    std::cout << "link_name: " << link_name << std::endl;
    std::cout << "center_of_mass: " << center_of_mass_ << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // ROS node
    node_ = gazebo_ros::Node::Get(sdf);

    if (!ns.empty()) {
      ns.append("/");
    }

    // ROS publishers
    flight_data_pub_ = node_->create_publisher<tello_msgs::msg::FlightData>(ns + "flight_data", 1);
    tello_response_pub_ = node_->create_publisher<tello_msgs::msg::TelloResponse>(ns + "tello_response", 1);

    // ROS service
    command_srv_ = node_->create_service<tello_msgs::srv::TelloAction>(ns + "tello_action",
      std::bind(&TelloPlugin::command_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // ROS subscription
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(ns + "cmd_vel",
      std::bind(&TelloPlugin::cmd_vel_callback, this, std::placeholders::_1));

    // 10Hz ROS timer
    timer_ = node_->create_wall_timer(100ms, std::bind(&TelloPlugin::spin_10Hz, this));

    // Listen for the Gazebo update event. This event is broadcast every simulation iteration.
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&TelloPlugin::OnUpdate, this, _1));
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const gazebo::common::UpdateInfo& /*info*/)
  {
    ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
    ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

    ignition::math::Vector3d force;
    // TODO compute force
    base_link_->AddLinkForce(force, center_of_mass_);

    ignition::math::Vector3d torque;
    // TODO compute torque
    base_link_->AddRelativeTorque(torque); // ODE adds torque at the center of mass
  }

  void command_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
    std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
  {
    // TODO initiate command
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    twist_ = *msg;
  }

  void spin_10Hz()
  {
    // TODO publish flight data
    // TODO publish tello response
  }
};

GZ_REGISTER_MODEL_PLUGIN(TelloPlugin)

} // namespace tello_gazebo
