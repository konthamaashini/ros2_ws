#include <gazebo/gazebo.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/PhysicsIface.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Dense>
#include <array>

namespace gazebo {

  class CustomDVLPlugin : public SensorPlugin
  {
  public:
    CustomDVLPlugin() : SensorPlugin() {}
    virtual ~CustomDVLPlugin() {}

    /// \brief Load the sensor plugin.
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
    {
      // Initialize ROS
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      node_ = rclcpp::Node::make_shared("custom_dvl_plugin");
      publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3>("dvl/velocity", 10);

      // Cast to RaySensor
      this->raySensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      if (!this->raySensor_) {
        gzerr << "CustomDVLPlugin requires a RaySensor.\n";
        return;
      }

      // Initialize beam indices and previous ranges
      beamIndices_ = {0, 1, 2, 3};
      for (size_t i = 0; i < beamIndices_.size(); ++i) {
        previousRanges_[i] = this->raySensor_->Range(beamIndices_[i]);
      }

      // Store initial update time
      last_update_time_ = this->raySensor_->LastUpdateTime();

      // Connect update event
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomDVLPlugin::OnUpdate, this));
    }

    /// \brief Called on every simulation update.
    void OnUpdate()
    {
      // Calculate time difference
      common::Time current_time = this->raySensor_->LastUpdateTime();
      double dt = (current_time - last_update_time_).Double();
      if (dt <= 0.0) return;
      last_update_time_ = current_time;

      // Calculate range rates for each beam
      Eigen::Vector4d d;
      for (size_t i = 0; i < beamIndices_.size(); ++i) {
        double currentRange = this->raySensor_->Range(beamIndices_[i]);
        double rangeRate = (previousRanges_[i] - currentRange) / dt; // Negative of (current - prev)/dt
        d(i) = rangeRate;
        previousRanges_[i] = currentRange;
      }

      // Define beam directions (20° from vertical, pointing downward)
      const double alpha = 0.3491; // 20° in radians
      Eigen::Matrix<double, 4, 3> N;
      N.row(0) = Eigen::Vector3d(  std::sin(alpha),  0, std::cos(alpha) ); // +X beam
      N.row(1) = Eigen::Vector3d(  0,  std::sin(alpha), std::cos(alpha) ); // +Y beam
      N.row(2) = Eigen::Vector3d( -std::sin(alpha),  0, std::cos(alpha) ); // -X beam
      N.row(3) = Eigen::Vector3d(  0, -std::sin(alpha), std::cos(alpha) ); // -Y beam

      // Solve N * v = -d for velocity v
      Eigen::Vector3d velocity = -(N.transpose() * N).inverse() * N.transpose() * d;

      // Publish velocity
      geometry_msgs::msg::Vector3 msg;
      msg.x = velocity.x();
      msg.y = velocity.y();
      msg.z = velocity.z();
      publisher_->publish(msg);

      // Process ROS callbacks
      rclcpp::spin_some(node_);
    }

  private:
    sensors::RaySensorPtr raySensor_;
    event::ConnectionPtr updateConnection_;
    common::Time last_update_time_;
    std::array<double, 4> previousRanges_;
    std::array<int, 4> beamIndices_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(CustomDVLPlugin)
}