#ifndef __UUV_DVL_ROS_PLUGIN_HH__
#define __UUV_DVL_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/range.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>
#include <uuv_sensor_ros_plugins_msgs/msg/dvl.hpp>
#include <uuv_sensor_ros_plugins_msgs/msg/dvl_beam.hpp>

// **NEW:** Include the header for tf2_ros::Buffer
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

#include "SensorDvl.pb.h"

#define ALTITUDE_OUT_OF_RANGE -1.0

namespace gazebo
{
  /// \brief DVLROSPlugin provides a ROS interface for a Doppler Velocity Log sensor.
  class DVLROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: DVLROSPlugin();

    /// \brief Class destructor
    public: virtual ~DVLROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Get beam Range message update
    protected: void OnBeamCallback(const sensor_msgs::msg::Range::ConstSharedPtr& _range0,
                                  const sensor_msgs::msg::Range::ConstSharedPtr& _range1,
                                  const sensor_msgs::msg::Range::ConstSharedPtr& _range2,
                                  const sensor_msgs::msg::Range::ConstSharedPtr& _range3);

    /// \brief Updates the poses of each beam with respect to the DVL frame
    protected: bool UpdateBeamTransforms();

    protected: bool beamTransformsInitialized;

    /// \brief Measured altitude in meters
    protected: double altitude;

    /// \brief ROS DVL message
    protected: uuv_sensor_ros_plugins_msgs::msg::DVL dvlROSMsg;

    protected: std::vector<uuv_sensor_ros_plugins_msgs::msg::DVLBeam> dvlBeamMsgs;

    /// \brief ROS publisher for twist data.
    protected: rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twistPub;

    /// \brief Stored twist message.
    protected: geometry_msgs::msg::TwistWithCovarianceStamped twistROSMsg;

    /// \brief List of beam links
    protected: std::vector<std::string> beamsLinkNames;

    /// \brief List of beam topics
    protected: std::vector<std::string> beamTopics;

    /// \brief List of poses of each beam with respect to the DVL frame
    protected: std::vector<ignition::math::Pose3d> beamPoses;

    protected: std::shared_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Range, sensor_msgs::msg::Range, sensor_msgs::msg::Range, sensor_msgs::msg::Range>>
      syncBeamMessages;

    protected: std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Range>> beamSub0;
    protected: std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Range>> beamSub1;
    protected: std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Range>> beamSub2;
    protected: std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Range>> beamSub3;

    // TF2 components for transform handling.
    protected: std::unique_ptr<tf2_ros::Buffer> myBuffer;
    protected: std::unique_ptr<tf2_ros::TransformListener> myTransformListener;

    /// \brief Publisher for transporting measurement messages.
    protected: rclcpp::Publisher<uuv_sensor_ros_plugins_msgs::msg::DVL>::SharedPtr rosSensorOutputPub;
  };
}

#endif // __UUV_DVL_ROS_PLUGIN_HH__
