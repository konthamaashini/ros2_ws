#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

namespace gazebo
{
class CaudalForcePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
  {
    this->model = _model;
    this->link = model->GetLink("link_base");

    if (!this->link)
    {
      gzerr << "[ERROR] Link 'link_base' not found in model! Plugin will not apply force.\n";
      return;
    }

    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->ros_node = std::make_shared<rclcpp::Node>("caudal_force_plugin");

    this->velocity_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
        "/caudal_velocity", 10,
        std::bind(&CaudalForcePlugin::OnVelocityMsg, this, std::placeholders::_1));

    this->vertical_force_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
        "/vertical_force_z", 10,
        std::bind(&CaudalForcePlugin::OnVerticalForceMsg, this, std::placeholders::_1));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CaudalForcePlugin::OnUpdate, this));

    last_msg_time = this->ros_node->get_clock()->now();

    gzmsg << "[INFO] CaudalForcePlugin loaded! Applying X and Z force on "
          << this->link->GetName() << "\n";
  }

  void OnVelocityMsg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->caudal_velocity = msg->data;
    last_msg_time = this->ros_node->get_clock()->now();
  }

  void OnVerticalForceMsg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->vertical_force_z = msg->data;
  }

  void OnUpdate()
  {
    if (!this->link)
    {
      gzerr << "[ERROR] Link is null. Force cannot be applied!\n";
      return;
    }

    auto now = this->ros_node->get_clock()->now();
    if ((now - last_msg_time).nanoseconds() * 1e-9 > 0.1)
    {
      caudal_velocity = 0.0;
      gzdbg << "[DEBUG] Timeout: No /caudal_velocity messages for 0.1s, reset caudal_velocity to 0\n";
    }

    // Compute caudal force in local x
    double k = 1.0;
    double force_x = k * std::abs(this->caudal_velocity) * caudal_velocity;

    // Apply both caudal and vertical force in local frame
    ignition::math::Vector3d force_local(force_x, 0, vertical_force_z);

    // Convert to world frame
    ignition::math::Quaterniond rot = this->link->WorldPose().Rot();
    ignition::math::Vector3d force_world = rot.RotateVector(force_local);

    ignition::math::Vector3d com = this->link->GetInertial()->Pose().Pos();
    this->link->AddForceAtRelativePosition(force_world, com);

    // Rotation damping
    double kp = 10.0;
    ignition::math::Vector3d angular_vel = this->link->WorldAngularVel();
    ignition::math::Vector3d counter_torque(
        -kp * angular_vel.X(),
        -kp * angular_vel.Y(),
        -kp * angular_vel.Z()
    );
    this->link->AddTorque(counter_torque);

    // Constrain linear velocity to x and z only
    ignition::math::Vector3d world_vel = this->link->WorldLinearVel();
    ignition::math::Vector3d local_vel = rot.RotateVectorReverse(world_vel);
    local_vel.Y() = 0; // prevent sideways drift
    ignition::math::Vector3d constrained_vel = rot.RotateVector(local_vel);
    this->link->SetLinearVel(constrained_vel);

    gzdbg << "[DEBUG] Time: " << now.nanoseconds() * 1e-9
          << " | Force local: " << force_local
          << " | Force world: " << force_world
          << " | Vertical Z force: " << vertical_force_z
          << " | Angular vel: " << angular_vel
          << " | Linear vel local: " << local_vel
          << " | Caudal velocity command: " << this->caudal_velocity << "\n";

    rclcpp::spin_some(this->ros_node);
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  event::ConnectionPtr updateConnection;

  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vertical_force_sub;

  double caudal_velocity = 0.0;
  double vertical_force_z = 0.0;
  rclcpp::Time last_msg_time;
};

GZ_REGISTER_MODEL_PLUGIN(gazebo::CaudalForcePlugin)
}
