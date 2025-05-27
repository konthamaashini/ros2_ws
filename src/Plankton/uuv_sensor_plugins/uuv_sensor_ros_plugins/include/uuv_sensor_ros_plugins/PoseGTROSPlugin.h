#ifndef __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__
#define __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Added to ensure that tf2_ros::Buffer is defined.
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo/physics/physics.hh>

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>

#include <memory>

namespace gazebo
{
  class PoseGTROSPlugin : public ROSBaseModelPlugin
  {
    public:
      PoseGTROSPlugin();
      ~PoseGTROSPlugin();

      /// \brief Load the plugin
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

      /// \brief Update sensor measurement
      virtual bool OnUpdate(const common::UpdateInfo& _info) override;

    protected:
      void PublishNEDOdomMessage(common::Time _time,
                                 ignition::math::Pose3d _pose, 
                                 ignition::math::Vector3d _linVel,
                                 ignition::math::Vector3d _angVel);

      void PublishOdomMessage(common::Time _time,
                              ignition::math::Pose3d _pose, 
                              ignition::math::Vector3d _linVel,
                              ignition::math::Vector3d _angVel);

      void UpdateNEDTransform();

    protected:
      /// \brief Publisher for NED odometry messages.
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nedOdomPub;

      /// \brief Pose offset.
      ignition::math::Pose3d offset;

      /// \brief NED frame ID.
      std::string nedFrameID;

      /// \brief NED transform.
      ignition::math::Pose3d nedTransform;

      /// \brief Flag to indicate whether the NED transform is initialized.
      bool nedTransformIsInit;

      /// \brief Whether to publish NED odometry.
      bool publishNEDOdom;

      // TF2 components for transform handling:
      std::unique_ptr<tf2_ros::Buffer> myTfBuffer;
      std::unique_ptr<tf2_ros::TransformListener> tfListener;

      /// \brief Last linear velocity.
      ignition::math::Vector3d lastLinVel;

      /// \brief Last angular velocity.
      ignition::math::Vector3d lastAngVel;

      /// \brief Linear acceleration.
      ignition::math::Vector3d linAcc;

      /// \brief Angular acceleration.
      ignition::math::Vector3d angAcc;

      /// \brief Last reference linear velocity.
      ignition::math::Vector3d lastRefLinVel;

      /// \brief Last reference angular velocity.
      ignition::math::Vector3d lastRefAngVel;

      /// \brief Reference linear acceleration.
      ignition::math::Vector3d refLinAcc;

      /// \brief Reference angular acceleration.
      ignition::math::Vector3d refAngAcc;

      /// \brief Publisher for transporting sensor measurement messages.
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rosSensorOutputPub;
  };
}

#endif // __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__
