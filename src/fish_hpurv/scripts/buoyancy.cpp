#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class AUVBuoyancyPlugin : public rclcpp::Node {
public:
    AUVBuoyancyPlugin() : Node("auv_buoyancy_plugin") {
        // Declare parameters
        declare_parameter("dry_mass", 13.0); // Dry mass (kg)
        declare_parameter("pitching_mass", 1.0); // Pitching mass (kg)
        declare_parameter("gravity", 9.81); // Gravity (m/s^2)
        declare_parameter("water_density", 1021.0); // Water density (kg/m^3)
        declare_parameter("nominal_volume", 0.0143); // Nominal volume (m^3)
        declare_parameter("max_volume", 0.02288); // Max volume (m^3)
        declare_parameter("min_volume", 0.010725); // Min volume (m^3)
        declare_parameter("target_depth", -5.0); // Target depth (m)
        declare_parameter("kp_depth", 0.001); // Proportional gain for depth control
        declare_parameter("kd_depth", 0.002); // Derivative gain for depth control
        declare_parameter("volume_rate_limit", 0.00001); // Max volume change rate (m^3/s)

        // Get parameters
        get_parameter("dry_mass", m_);
        get_parameter("pitching_mass", m_p_);
        get_parameter("gravity", g_);
        get_parameter("water_density", rho_);
        get_parameter("nominal_volume", V_nominal_);
        get_parameter("max_volume", V_max_);
        get_parameter("min_volume", V_min_);
        get_parameter("target_depth", z_target_);
        get_parameter("kp_depth", kp_depth_);
        get_parameter("kd_depth", kd_depth_);
        get_parameter("volume_rate_limit", volume_rate_limit_);

        // Initialize variables
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

        // Subscribers
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "vehicle/pose", 10, std::bind(&AUVBuoyancyPlugin::poseCallback, this, std::placeholders::_1));
        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "vehicle/twist", 10, std::bind(&AUVBuoyancyPlugin::twistCallback, this, std::placeholders::_1));

        // Publishers
        buoyancy_force_pub_ = create_publisher<std_msgs::msg::Float64>("vehicle/buoyancy_force_z", 10);
        forward_volume_pub_ = create_publisher<std_msgs::msg::Float64>("vehicle/forward_volume", 10);
        aft_volume_pub_ = create_publisher<std_msgs::msg::Float64>("vehicle/aft_volume", 10);

        // Timer for control loop (200 Hz, dt = 0.005 s)
        timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&AUVBuoyancyPlugin::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "AUV Buoyancy Plugin initialized");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        z_ = msg->position.z;
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        w_ = msg->linear.z; // Vertical velocity
    }

    void controlLoop() {
        double dt = 0.005; // Time step (s)

        // Depth control using PID
        double depth_error = z_target_ - z_;
        double volume_adjustment = kp_depth_ * depth_error - kd_depth_ * w_;
        
        // Limit volume adjustment rate
        volume_adjustment = std::max(std::min(volume_adjustment, volume_rate_limit_ * dt), -volume_rate_limit_ * dt);

        // Adjust volumes symmetrically
        V_forward_ += volume_adjustment / 2.0;
        V_aft_ += volume_adjustment / 2.0;

        // Enforce volume limits
        V_forward_ = std::max(std::min(V_forward_, V_forward_max_), V_forward_min_);
        V_aft_ = std::max(std::min(V_aft_, V_aft_max_), V_aft_min_);

        // Compute buoyancy force
        double F_b = rho_ * g_ * (V_forward_ + V_aft_ - V_nominal_);

        // Publish buoyancy force
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

        // Log for debugging
        RCLCPP_DEBUG(this->get_logger(),
            "t=%.2f s, z=%.3f m, z_target=%.3f m, V_forward=%.6f m^3, V_aft=%.6f m^3, F_b=%.2f N",
            this->now().seconds(), z_, z_target_, V_forward_, V_aft_, F_b);
    }

    // Parameters
    double m_, m_p_, m_total_dry_, g_, rho_;
    double V_nominal_, V_max_, V_min_;
    double V_forward_nominal_, V_aft_nominal_;
    double V_forward_max_, V_aft_max_, V_forward_min_, V_aft_min_;
    double z_target_, kp_depth_, kd_depth_, volume_rate_limit_;

    // State variables
    double V_forward_, V_aft_;
    double z_, w_;

    // ROS2 components
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr buoyancy_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr forward_volume_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr aft_volume_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AUVBuoyancyPlugin>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}