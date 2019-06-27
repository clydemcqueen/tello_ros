#include <chrono>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tello_msgs/srv/tello_action.hpp"

#include "pid.hpp"

using namespace std::chrono_literals;

//===============================================================================================
// TelloPlugin features:
// -- generates trivial flight data at 10Hz
// -- video is managed by a gazebo_ros_pkgs plugin, see tello_description/urdf/tello.xml
// -- responds to "takeoff" and "land" commands
// -- responds to cmd_vel and "rc x y z yaw" commands
// -- battery state
//
// Tello flight dynamics are sophisticated and difficult to model. TelloPlugin keeps it simple:
// -- x, y, z and yaw velocities are controlled by a P controller
// -- roll and pitch are always 0
//
// Possible extensions:
// -- simulate network latency
// -- add some randomness to the flight dynamics
// -- improve battery simulation
//===============================================================================================

namespace tello_gazebo
{

  const double MAX_XY_V = 8.0;
  const double MAX_Z_V = 4.0;
  const double MAX_ANG_V = M_PI;

  const double MAX_XY_A = 8.0;
  const double MAX_Z_A = 4.0;
  const double MAX_ANG_A = M_PI;

  const double TAKEOFF_Z = 1.0;       // Takeoff target z position
  const double TAKEOFF_Z_V = 0.5;     // Takeoff target z velocity

  const double LAND_Z = 0.1;          // Land target z position
  const double LAND_Z_V = -0.5;       // Land target z velocity

  const int BATTERY_DURATION = 6000;  // Battery duration in seconds

  inline double clamp(const double v, const double max)
  {
    return v > max ? max : (v < -max ? -max : v);
  }

  class TelloPlugin : public gazebo::ModelPlugin
  {
    enum class FlightState
    {
      landed,
      taking_off,
      flying,
      landing,
      dead_battery,
    };

    std::map<FlightState, const char *> state_strs_{
      {FlightState::landed,       "landed"},
      {FlightState::taking_off,   "taking_off"},
      {FlightState::flying,       "flying"},
      {FlightState::landing,      "landing"},
      {FlightState::dead_battery, "dead_battery"},
    };

    FlightState flight_state_;

    gazebo::physics::LinkPtr base_link_;
    ignition::math::Vector3d gravity_;
    ignition::math::Vector3d center_of_mass_{0, 0, 0};
    int battery_duration_{BATTERY_DURATION};

    // Connection to Gazebo message bus
    gazebo::event::ConnectionPtr update_connection_;

    // GazeboROS node
    gazebo_ros::Node::SharedPtr node_;

    // ROS publishers
    rclcpp::Publisher<tello_msgs::msg::FlightData>::SharedPtr flight_data_pub_;
    rclcpp::Publisher<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_pub_;

    // ROS services
    rclcpp::Service<tello_msgs::srv::TelloAction>::SharedPtr command_srv_;

    // ROS subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Sim time of last update
    gazebo::common::Time update_time_;

    // 10Hz timer
    gazebo::common::Time ten_hz_time_;

    // cmd_vel messages control x, y, z and yaw velocity
    pid::Controller x_controller_{false, 2, 0, 0};
    pid::Controller y_controller_{false, 2, 0, 0};
    pid::Controller z_controller_{false, 2, 0, 0};
    pid::Controller yaw_controller_{false, 2, 0, 0};

  public:

    TelloPlugin()
    {
      (void) update_connection_;
      (void) command_srv_;
      (void) cmd_vel_sub_;

      transition(FlightState::landed);
    }

    ~TelloPlugin()
    {
    }

    void set_target_velocities(double x, double y, double z, double yaw)
    {
      x_controller_.set_target(x);
      y_controller_.set_target(y);
      z_controller_.set_target(z);
      yaw_controller_.set_target(yaw);
    }

    // Respond to a command of the form "rc x y z yaw"
    void set_target_velocities(std::string rc_command)
    {
      double x, y, z, yaw;

      try {
        std::istringstream iss(rc_command, std::istringstream::in);
        std::string s;
        iss >> s; // "rc"
        iss >> s;
        x = std::stof(s);
        iss >> s;
        y = std::stof(s);
        iss >> s;
        z = std::stof(s);
        iss >> s;
        yaw = std::stof(s);
      } catch (std::exception e) {
        RCLCPP_ERROR(node_->get_logger(), "can't parse rc command '%s', exception %s", rc_command.c_str(), e.what());
        return;
      }

      set_target_velocities(
        x * MAX_XY_V,
        y * MAX_XY_V,
        z * MAX_Z_V,
        yaw * MAX_ANG_V);
    }

    void transition(FlightState next)
    {
      if (node_ != nullptr) {
        RCLCPP_INFO(node_->get_logger(), "transition from '%s' to '%s'", state_strs_[flight_state_], state_strs_[next]);
      }

      flight_state_ = next;

      switch (flight_state_) {
        case FlightState::landed:
        case FlightState::flying:
        case FlightState::dead_battery:
          set_target_velocities(0, 0, 0, 0);
          break;

        case FlightState::taking_off:
          set_target_velocities(0, 0, TAKEOFF_Z_V, 0);
          break;

        case FlightState::landing:
          set_target_velocities(0, 0, LAND_Z_V, 0);
          break;
      }
    }

    // Called once when the plugin is loaded.
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      GZ_ASSERT(model != nullptr, "Model is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      std::string link_name{"base_link"};

      // In theory we can move much of this config into the <ros> tag, but this appears unfinished in Dashing?
      if (sdf->HasElement("link_name")) {
        link_name = sdf->GetElement("link_name")->Get<std::string>();
      }
      if (sdf->HasElement("center_of_mass")) {
        center_of_mass_ = sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
      }
      if (sdf->HasElement("battery_duration")) {
        battery_duration_ = sdf->GetElement("center_of_mass")->Get<int>();
      }

      base_link_ = model->GetLink(link_name);
      GZ_ASSERT(base_link_ != nullptr, "Missing link");
      gravity_ = model->GetWorld()->Gravity();

      std::cout << std::fixed;
      std::setprecision(2);
      std::cout << std::endl;
      std::cout << "TELLO PLUGIN" << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
      std::cout << "link_name: " << link_name << std::endl;
      std::cout << "center_of_mass: " << center_of_mass_ << std::endl;
      std::cout << "gravity: " << gravity_ << std::endl;
      std::cout << "battery_duration: " << battery_duration_ << std::endl;
      std::cout << "-----------------------------------------" << std::endl;
      std::cout << std::endl;

      // ROS node
      node_ = gazebo_ros::Node::Get(sdf);

      // Fix by adding <parameter name="use_sim_time" type="bool">1</parameter> to the SDF file
      bool use_sim_time;
      node_->get_parameter("use_sim_time", use_sim_time);
      if (!use_sim_time) {
        RCLCPP_ERROR(node_->get_logger(), "use_sim_time is false, could be a bug");
      }

      // ROS publishers
      flight_data_pub_ = node_->create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);
      tello_response_pub_ = node_->create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 1);

      // ROS service
      command_srv_ = node_->create_service<tello_msgs::srv::TelloAction>("tello_action",
                                                                         std::bind(&TelloPlugin::command_callback, this,
                                                                                   std::placeholders::_1,
                                                                                   std::placeholders::_2,
                                                                                   std::placeholders::_3));

      // ROS subscription
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                           std::bind(&TelloPlugin::cmd_vel_callback,
                                                                                     this, std::placeholders::_1));

      // Listen for the Gazebo update event. This event is broadcast every simulation iteration.
      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&TelloPlugin::OnUpdate, this, _1));
    }

    // Called by the world update start event, up to 1000 times per second.
    void OnUpdate(const gazebo::common::UpdateInfo &info)
    {
      // Do nothing if the battery is dead
      if (flight_state_ == FlightState::dead_battery) {
        return;
      }

      // 10Hz timer
      if ((info.simTime - ten_hz_time_).Double() > 0.1) {
        spin_10Hz();
        ten_hz_time_ = info.simTime;
      }

      // dt
      double dt = (info.simTime - update_time_).Double();
      update_time_ = info.simTime;

      // Don't apply force if we're landed
      if (flight_state_ != FlightState::landed) {
        // Get current velocity
        ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
        ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

        // Calc desired acceleration (ubar)
        ignition::math::Vector3d lin_ubar, ang_ubar;
        lin_ubar.X(x_controller_.calc(linear_velocity.X(), dt, 0));
        lin_ubar.Y(y_controller_.calc(linear_velocity.Y(), dt, 0));
        lin_ubar.Z(z_controller_.calc(linear_velocity.Z(), dt, 0));
        ang_ubar.Z(yaw_controller_.calc(angular_velocity.Z(), dt, 0));

        // Clamp acceleration
        lin_ubar.X() = clamp(lin_ubar.X(), MAX_XY_A);
        lin_ubar.Y() = clamp(lin_ubar.Y(), MAX_XY_A);
        lin_ubar.Z() = clamp(lin_ubar.Z(), MAX_Z_A);
        ang_ubar.Z() = clamp(ang_ubar.Z(), MAX_ANG_A);

        // Compensate for gravity
        lin_ubar -= gravity_;

        // Calc force and torque
        ignition::math::Vector3d force = lin_ubar * base_link_->GetInertial()->Mass();
        ignition::math::Vector3d torque = ang_ubar * base_link_->GetInertial()->MOI();

        // Set roll and pitch to 0
        ignition::math::Pose3d pose = base_link_->WorldPose();
        pose.Rot().X(0);
        pose.Rot().Y(0);
        base_link_->SetWorldPose(pose);

        // Apply force and torque
        base_link_->AddLinkForce(force, center_of_mass_);
        base_link_->AddRelativeTorque(torque); // ODE adds torque at the center of mass
      }
    }

    bool is_prefix(const std::string &prefix, const std::string &str)
    {
      return std::equal(prefix.begin(), prefix.end(), str.begin());
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"

    void command_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
      std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
    {
      if (request->cmd == "takeoff" && flight_state_ == FlightState::landed) {
        transition(FlightState::taking_off);
        response->rc = response->OK;
      } else if (request->cmd == "land" && flight_state_ == FlightState::flying) {
        transition(FlightState::landing);
        response->rc = response->OK;
      } else if (is_prefix("rc", request->cmd) && flight_state_ == FlightState::flying) {
        set_target_velocities(request->cmd);
        response->rc = response->OK;
      } else {
        RCLCPP_WARN(node_->get_logger(), "ignoring command '%s'", request->cmd.c_str());
        response->rc = response->ERROR_BUSY;
      }
    }

#pragma clang diagnostic pop

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      if (flight_state_ == FlightState::flying) {
        // TODO cmd_vel should specify velocity, not joystick position
        set_target_velocities(
          msg->linear.x * MAX_XY_V,
          msg->linear.y * MAX_XY_V,
          msg->linear.z * MAX_Z_V,
          msg->angular.z * MAX_ANG_V);
      }
    }

    void respond_ok()
    {
      tello_msgs::msg::TelloResponse msg;
      msg.rc = msg.OK;
      msg.str = "ok";
      tello_response_pub_->publish(msg);
    }

    void spin_10Hz()
    {
      rclcpp::Time ros_time = node_->now();

      // Wait for ROS time to get reasonable TODO sometimes this never happens
      if (ros_time.seconds() < 1.0) {
        return;
      }

      // Simulate a battery
      int battery_percent = static_cast<int>((battery_duration_ - ros_time.seconds()) / battery_duration_ * 100);
      if (battery_percent <= 0) {
        // We're dead
        transition(FlightState::dead_battery);
        return;
      }

      // Publish flight data
      tello_msgs::msg::FlightData flight_data;
      flight_data.header.stamp = ros_time;
      flight_data.sdk = flight_data.SDK_1_3;
      flight_data.bat = battery_percent;
      flight_data_pub_->publish(flight_data);

      // Finish pending actions
      ignition::math::Pose3d pose = base_link_->WorldPose();
      if (flight_state_ == FlightState::taking_off && pose.Pos().Z() > TAKEOFF_Z) {
        transition(FlightState::flying);
        respond_ok();
      } else if (flight_state_ == FlightState::landing && pose.Pos().Z() < LAND_Z) {
        transition(FlightState::landed);
        respond_ok();
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(TelloPlugin)

} // namespace tello_gazebo
