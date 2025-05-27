#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class ForceDynamicNode : public rclcpp::Node
{
public:
    ForceDynamicNode() : Node("force_dynamic_node")
    {
        // Set use_sim_time
        set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Initialize model parameters (aligned with URDF)
        m_ = 12.6; // Total mass
        rho_ = 1000.0; // Density of water
        g_ = 9.81; // Gravity
        a_ = 0.17; b_ = 0.24; L_ = 0.740; // Dimensions
        xG_ = 0.0325815; yG_ = 0.0493006; zG_ = -0.0204279; // CoG
        xB_ = 0.0325815; yB_ = 0.0493006; zB_ = 0.02; // CoB
        IxG_ = 0.01; IyG_ = 0.01; IzG_ = 0.01; // MoI
        Ixy_ = 0.0; Iyz_ = 0.0; Ixz_ = 0.0;
        Ix_ = IxG_ + m_ * (yG_ * yG_ + zG_ * zG_);
        Iy_ = IyG_ + m_ * (xG_ * xG_ + zG_ * zG_);
        Iz_ = IzG_ + m_ * (xG_ * xG_ + yG_ * yG_);

        // Fin positions (aligned with URDF)
        x1_ = -0.425323; y1_ = 0.0; z1_ = -0.06; // caudal_joint
        x2_ = 0.05; y2_ = -0.18; z2_ = -0.035; // joint1
        x3_ = 0.05; y3_ = 0.08; z3_ = -0.035; // joint2

        Cl_ = 0.92; Cd_ = 1.12;
        S1_ = 0.024; L_f1_ = 0.2;
        S2_ = 0.044; L_f2_ = 0.1;
        S3_ = 0.044; L_f3_ = 0.1;
        PF1max_ = 5.0;
        freqmax_ = 2.0;
        dt_ = 0.033; // Match controller update rate (30 Hz)
        Ux_ = 0.4; Uy_ = 0.4; Uz_ = 0.0;

        // Buoyancy parameters from AUVBuoyancyPlugin
        declare_parameter("pitching_mass", 1.0); // Pitching mass (kg)
        declare_parameter("water_density", 1021.0); // Water density (kg/m^3)
        declare_parameter("nominal_volume", 0.0143); // Nominal volume (m^3)
        declare_parameter("max_volume", 0.02288); // Max volume (m^3)
        declare_parameter("min_volume", 0.010725); // Min volume (m^3)
        declare_parameter("target_depth", -5.0); // Target depth (m)
        declare_parameter("kp_depth", 0.001); // Proportional gain for depth control
        declare_parameter("kd_depth", 0.002); // Derivative gain for depth control
        declare_parameter("volume_rate_limit", 0.00001); // Max volume change rate (m^3/s)

        // Get buoyancy parameters
        get_parameter("pitching_mass", m_p_);
        get_parameter("water_density", rho_); // Override default rho_
        get_parameter("nominal_volume", V_nominal_);
        get_parameter("max_volume", V_max_);
        get_parameter("min_volume", V_min_);
        get_parameter("target_depth", z_target_);
        get_parameter("kp_depth", kp_depth_);
        get_parameter("kd_depth", kd_depth_);
        get_parameter("volume_rate_limit", volume_rate_limit_);

        // Initialize buoyancy variables
        m_total_dry_ = m_ + m_p_;
        V_forward_nominal_ = V_nominal_ / 2.0;
        V_aft_nominal_ = V_nominal_ / 2.0;
        V_forward_max_ = V_max_ / 2.0;
        V_aft_max_ = V_max_ / 2.0;
        V_forward_min_ = V_min_ / 2.0;
        V_aft_min_ = V_min_ / 2.0;
        V_forward_ = V_forward_nominal_;
        V_aft_ = V_aft_nominal_;
        z_ = 0.0;
        w_ = 0.0;

        // Initialize state vectors
        nu_ = Eigen::VectorXd::Zero(6);
        eta_ = Eigen::VectorXd::Zero(6);
        eta_(0) = 0.0; eta_(1) = 0.0; eta_(2) = 0.0;
        eta_(3) = 0.0; eta_(4) = 0.0; eta_(5) = 0.0;

        // Fin angles
        b1_ = 0.0; b2_ = 0.0; b3_ = 0.0;

        // ROS 2 subscriptions and publications
        sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10,
            std::bind(&ForceDynamicNode::OnJointTrajectory, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/hpurv/pose", 10, std::bind(&ForceDynamicNode::poseCallback, this, std::placeholders::_1));

        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/hpurv/twist", 10, std::bind(&ForceDynamicNode::twistCallback, this, std::placeholders::_1));

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
            "/hpurv/pose", 10);
        trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        velocity_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/caudal_velocity", 10);
        buoyancy_force_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/vertical_force_z", 10);
        forward_volume_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/vehicle/forward_volume", 10);
        aft_volume_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/vehicle/aft_volume", 10);

        // Timer for dynamics update (30 Hz)
        timer_ = create_wall_timer(
            33ms, std::bind(&ForceDynamicNode::TimerCallback, this));

        RCLCPP_INFO(get_logger(), "Force Dynamic Node with Buoyancy initialized");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        z_ = msg->position.z;
        eta_(0) = msg->position.x;
        eta_(1) = msg->position.y;
        eta_(2) = msg->position.z;
        // Update orientation (eta_(3:5)) if needed
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        w_ = msg->linear.z; // Vertical velocity
        nu_(0) = msg->linear.x;
        nu_(1) = msg->linear.y;
        nu_(2) = msg->linear.z;
    }

    void OnJointTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (!msg->points.empty() && msg->points[0].positions.size() == 3)
        {
            b1_ = std::max(-0.523599, std::min(0.523599, msg->points[0].positions[0]));
            b2_ = std::max(-0.785398, std::min(0.785398, msg->points[0].positions[1]));
            b3_ = std::max(-0.785398, std::min(0.785398, msg->points[0].positions[2]));
            RCLCPP_INFO(get_logger(), "Received trajectory: b1=%f, b2=%f, b3=%f", b1_, b2_, b3_);
        }
    }

    void TimerCallback()
    {
        // Use simulation time
        double t = get_clock()->now().nanoseconds() * 1e-9; // Convert to seconds

        // Sinusoidal flapping trajectory
        double fi = 1.0; // 1 Hz for smooth motion
        b1_ = 0.5 * sin(2 * M_PI * fi * t); // Caudal fin
        b2_ = 0.5 * sin(2 * M_PI * fi * t + M_PI / 2); // Pectoral left
        b3_ = 0.5 * sin(2 * M_PI * fi * t + M_PI / 2); // Pectoral right
        b1_ = std::max(-0.523599, std::min(0.523599, b1_));
        b2_ = std::max(-0.785398, std::min(0.785398, b2_));
        b3_ = std::max(-0.785398, std::min(0.785398, b3_));

        // Compute velocities
        double v1 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t);
        double v2 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t + M_PI / 2);
        double v3 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t + M_PI / 2);

        // Buoyancy control (from AUVBuoyancyPlugin)
        double dt_buoyancy = 0.033; // Match timer rate (30 Hz)
        double depth_error = z_target_ - z_;
        double volume_adjustment = kp_depth_ * depth_error - kd_depth_ * w_;
        volume_adjustment = std::max(std::min(volume_adjustment, volume_rate_limit_ * dt_buoyancy), -volume_rate_limit_ * dt_buoyancy);

        // Adjust volumes symmetrically
        V_forward_ += volume_adjustment / 2.0;
        V_aft_ += volume_adjustment / 2.0;

        // Enforce volume limits
        V_forward_ = std::max(std::min(V_forward_, V_forward_max_), V_forward_min_);
        V_aft_ = std::max(std::min(V_aft_, V_aft_max_), V_aft_min_);

        // Compute buoyancy force
        double F_b = rho_ * g_ * (V_forward_ + V_aft_ - V_nominal_);

        // Publish buoyancy force to /vertical_force_z
        std_msgs::msg::Float64 force_msg;
        force_msg.data = F_b;
        buoyancy_force_pub_->publish(force_msg);

        // Publish buoyancy volumes for monitoring
        std_msgs::msg::Float64 forward_volume_msg;
        forward_volume_msg.data = V_forward_;
        forward_volume_pub_->publish(forward_volume_msg);

        std_msgs::msg::Float64 aft_volume_msg;
        aft_volume_msg.data = V_aft_;
        aft_volume_pub_->publish(aft_volume_msg);

        // Compute thrust force (original ForceDynamicNode logic)
        double k = 5.0; // Thrust coefficient (N·s/rad²)
        double force_x = k * std::abs(v1) * v1;
        double acc_x = force_x / m_; // Acceleration (m/s²)
        nu_(0) += acc_x * dt_; // Update velocity (m/s)
        eta_(0) += nu_(0) * dt_; // Update position (m)

        // Publish joint states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.name = {"caudal_joint", "joint1", "joint2"};
        joint_state.position = {b1_, b2_, b3_};
        joint_state.velocity = {v1, v2, v3};
        joint_state_pub_->publish(joint_state);

        // Publish joint trajectory commands
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = {"caudal_joint", "joint1", "joint2"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {b1_, b2_, b3_};
        point.velocities = {v1, v2, v3};
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        trajectory_msg.points.push_back(point);
        trajectory_pub_->publish(trajectory_msg);

        // Publish caudal velocity
        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = v1;
        velocity_pub_->publish(velocity_msg);

        // Publish pose
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = eta_(0);
        pose_msg.position.y = eta_(1);
        pose_msg.position.z = eta_(2);
        double roll = eta_(3), pitch = eta_(4), yaw = eta_(5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        pose_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        pose_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        pose_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        pose_msg.orientation.z = cr * cp * sy - sr * sp * cy;
        pose_pub_->publish(pose_msg);

        // Log for debugging
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Simulation time: %f", t);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fin angles: b1=%f, b2=%f, b3=%f", b1_, b2_, b3_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fin velocities: v1=%f, v2=%f, v3=%f", v1, v2, v3);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Estimated force: %f N", force_x);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Buoyancy: z=%f m, z_target=%f m, V_forward=%f m^3, V_aft=%f m^3, F_b=%f N", z_, z_target_, V_forward_, V_aft_, F_b);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Pose: x=%f, y=%f, z=%f", eta_(0), eta_(1), eta_(2));
    }

    // Model parameters (original)
    double m_, rho_, g_, a_, b_, L_;
    double xG_, yG_, zG_, xB_, yB_, zB_;
    double IxG_, IyG_, IzG_, Ixy_, Iyz_, Ixz_, Ix_, Iy_, Iz_;
    double x1_, y1_, z1_, x2_, y2_, z2_, x3_, y3_, z3_;
    double Cl_, Cd_, S1_, L_f1_, S2_, L_f2_, S3_, L_f3_;
    double PF1max_, freqmax_, dt_;
    double Ux_, Uy_, Uz_;
    double b1_, b2_, b3_;

    // Buoyancy parameters
    double m_p_, m_total_dry_;
    double V_nominal_, V_max_, V_min_;
    double V_forward_nominal_, V_aft_nominal_;
    double V_forward_max_, V_aft_max_, V_forward_min_, V_aft_min_;
    double z_target_, kp_depth_, kd_depth_, volume_rate_limit_;
    double V_forward_, V_aft_;
    double z_, w_;

    // Matrices and vectors
    Eigen::MatrixXd nu_, eta_;

    // ROS 2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr buoyancy_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr forward_volume_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr aft_volume_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceDynamicNode>());
    rclcpp::shutdown();
    return 0;
}