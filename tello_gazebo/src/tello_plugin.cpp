#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace tello_gazebo {

class TelloPlugin : public gazebo::ModelPlugin
{
  //gazebo::physics::Model model_;
  gazebo::physics::LinkPtr base_link_;

  // Force will be applied to the center_of_mass_ (body frame)
  // TODO is it possible to get this from the link?
  ignition::math::Vector3d center_of_mass_ {0, 0, 0};

  // Connection to Gazebo message bus
  gazebo::event::ConnectionPtr update_connection_;

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

    std::string link_name {"base_link"};

    std::cout << std::endl;
    std::cout << "TELLO PLUGIN" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default center of mass: " << center_of_mass_ << std::endl;

    if (sdf->HasElement("link_name")) {
      link_name = sdf->GetElement("link_name")->Get<std::string>();
      std::cout << "Link name: " << link_name << std::endl;
    }

    if (sdf->HasElement("center_of_mass")) {
      center_of_mass_ = sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
      std::cout << "Center of mass: " << center_of_mass_ << std::endl;
    }

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&TelloPlugin::OnUpdate, this, _1));

    // TODO provide tello_action service
    // TODO subscribe to cmd_vel
    // TODO publish tello_response
    // TODO publish flight_data
    // TODO publish image_raw
    // TODO publish camera_info
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
};

GZ_REGISTER_MODEL_PLUGIN(TelloPlugin)

} // namespace tello_gazebo
