#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "angles/angles.h"

#include "the_cruiser/implementations/catmull_rom_generator.hpp"
#include "the_cruiser/implementations/passive_safety_strategy.hpp"

using namespace std::chrono_literals;

class CruiserNode : public rclcpp::Node {
public:
    CruiserNode() : Node("cruiser_node") {
        // Inject the algorithm implementations
        trajectory_generator_ = std::make_shared<the_cruiser::implementations::CatmullRomGenerator>();
        safety_strategy_ = std::make_shared<the_cruiser::implementations::PassiveSafetyStrategy>();

        this->declare_parameter("max_v", 0.7);
        this->declare_parameter("max_w", 1.5);
        this->declare_parameter("kp_lin", 1.2);
        this->declare_parameter("kp_ang", 2.0);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/cruiser/smoothed_path", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CruiserNode::odom_callback, this, std::placeholders::_1));

        global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&CruiserNode::plan_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&CruiserNode::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&CruiserNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "CRUISER ONLINE. Ready to Smooth.");
    }

private:
    std::shared_ptr<the_cruiser::interfaces::ITrajectoryGenerator> trajectory_generator_;
    std::shared_ptr<the_cruiser::interfaces::ISafetyStrategy> safety_strategy_;
    
    std::vector<the_cruiser::types::TrajectoryPoint> trajectory_;
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    bool odom_received_ = false;
    bool path_active_ = false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Time start_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;

        // LOGGING 1: PROOF WE RECEIVED RAW DATA
        RCLCPP_WARN(this->get_logger(), ">>> [INPUT] Nav2 Raw Plan Received: %zu waypoints (Jagged)", msg->poses.size());

        std::vector<geometry_msgs::msg::Point> points;
        for(const auto& pose : msg->poses) points.push_back(pose.pose.position);

        // GENERATE
        trajectory_ = trajectory_generator_->smooth(points);
        
        // LOGGING 2: PROOF WE DID MATH (Upsampling)
        RCLCPP_INFO(this->get_logger(), ">>> [OUTPUT] Catmull-Rom Spline Generated: %zu points (Smooth)", trajectory_.size());
        RCLCPP_INFO(this->get_logger(), ">>> [PHYSICS] Estimated Travel Time: %.2f seconds", trajectory_.back().t);

        nav_msgs::msg::Path smooth_msg;
        smooth_msg.header = msg->header;
        smooth_msg.header.stamp = this->now(); // Update timestamp to now
        for(const auto& pt : trajectory_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.pose.position.x = pt.x;
            ps.pose.position.y = pt.y;
            smooth_msg.poses.push_back(ps);
        }
        path_pub_->publish(smooth_msg);

        start_time_ = this->now();
        path_active_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, th_);
        odom_received_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    void control_loop() {
        if (!odom_received_ || !path_active_ || trajectory_.empty()) return;
        
        const double goal_reached_wait_time = 1.0; // seconds
        double elapsed = (this->now() - start_time_).seconds();
        
        // Check if finished
        if (elapsed > trajectory_.back().t + goal_reached_wait_time) {
            stop_robot(); path_active_ = false;
            RCLCPP_WARN(this->get_logger(), ">>> [COMPLETE] Goal Reached. Final Error: %.3fm", 0.0); // Simple log
            return;
        }

        auto target = trajectory_.back();
        for (const auto& pt : trajectory_) { if (pt.t >= elapsed) { target = pt; break; } }

        double dx = target.x - x_;
        double dy = target.y - y_;
        double dist_err = std::hypot(dx, dy);
        double ang_err = angles::normalize_angle(std::atan2(dy, dx) - th_);

        double v_cmd = this->get_parameter("kp_lin").as_double() * dist_err;
        if (std::abs(ang_err) > 1.0) v_cmd = 0.0;
        double w_cmd = this->get_parameter("kp_ang").as_double() * ang_err;
        
        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        v_cmd = std::max(std::min(v_cmd, max_v), -max_v);
        w_cmd = std::max(std::min(w_cmd, max_w), -max_w);

        // --- Apply Safety Strategy ---
        if (last_scan_ && safety_strategy_) {
            auto safety_command = safety_strategy_->calculate_safety(last_scan_);
            v_cmd *= safety_command.speed_scale;
        }

        static int log_counter = 0;
        if (log_counter++ % 20 == 0) {
             RCLCPP_INFO(this->get_logger(), "Tracking: Time=%.1fs | V_cmd=%.2f m/s | DistErr=%.3fm", elapsed, v_cmd, dist_err);
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v_cmd; cmd.angular.z = w_cmd;
        cmd_vel_pub_->publish(cmd);
    }
    
    void stop_robot() {
        geometry_msgs::msg::Twist cmd; cmd.linear.x = 0; cmd.angular.z = 0;
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CruiserNode>());
    rclcpp::shutdown();
    return 0;
}
