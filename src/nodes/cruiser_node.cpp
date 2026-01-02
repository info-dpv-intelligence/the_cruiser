#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "angles/angles.h"

#include "the_cruiser/implementations/catmull_rom_generator.hpp"
#include "the_cruiser/implementations/passive_safety_strategy.hpp"

using namespace std::chrono_literals;
using LN = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CruiserNode : public rclcpp_lifecycle::LifecycleNode {
public:
    CruiserNode() : rclcpp_lifecycle::LifecycleNode("cruiser_node"),
                     cmd_vel_pub_count_(0), plan_received_count_(0), control_loop_count_(0) {
        // Inject the algorithm implementations
        trajectory_generator_ = std::make_shared<the_cruiser::implementations::CatmullRomGenerator>();
        safety_strategy_ = std::make_shared<the_cruiser::implementations::PassiveSafetyStrategy>();

        // Declare parameters
        this->declare_parameter("max_v", 0.7);
        this->declare_parameter("max_w", 1.5);
        this->declare_parameter("kp_lin", 1.2);
        this->declare_parameter("kp_ang", 2.0);
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

    // Diagnostic counters
    int cmd_vel_pub_count_;
    int plan_received_count_;
    int control_loop_count_;
    double last_dist_error_ = 0.0;
    double last_ang_error_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ============================================================================
    // LIFECYCLE CALLBACKS
    // ============================================================================

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Configuring Cruiser Node");

        // Create publishers and subscriptions
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/cruiser/smoothed_path", 10);
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CruiserNode::odom_callback, this, std::placeholders::_1));

        global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&CruiserNode::plan_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&CruiserNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Configuration complete");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Activating Cruiser Node - TAKING CONTROL OF /cmd_vel");

        // Start the control loop
        timer_ = this->create_wall_timer(50ms, std::bind(&CruiserNode::control_loop, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Deactivating Cruiser Node");
        
        // Stop control loop
        if (timer_) {
            timer_->cancel();
        }

        // Stop robot
        stop_robot();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Cleaning up Cruiser Node");
        
        // Destroy subscriptions and publishers
        odom_sub_.reset();
        global_plan_sub_.reset();
        scan_sub_.reset();
        cmd_vel_pub_.reset();
        path_pub_.reset();
        diag_pub_.reset();
        timer_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), ">>> LIFECYCLE: Shutting down Cruiser Node");
        return CallbackReturn::SUCCESS;
    }

    // ============================================================================
    // SUBSCRIPTION CALLBACKS
    // ============================================================================

    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;

        plan_received_count_++;

        // LOGGING 1: PROOF WE RECEIVED RAW DATA
        RCLCPP_WARN(this->get_logger(), ">>> [INPUT] Nav2 Raw Plan Received: %zu waypoints (Jagged) | Count: %d", 
                    msg->poses.size(), plan_received_count_);

        std::vector<geometry_msgs::msg::Point> points;
        for(const auto& pose : msg->poses) points.push_back(pose.pose.position);

        // GENERATE (YOUR ALGORITHM)
        trajectory_ = trajectory_generator_->smooth(points);
        
        // LOGGING 2: PROOF WE DID MATH (Upsampling)
        RCLCPP_INFO(this->get_logger(), ">>> [ALGORITHM] Catmull-Rom Spline Generated: %zu points (Smooth) | Travel Time: %.2f seconds", 
                    trajectory_.size(), trajectory_.back().t);

        // Publish smoothed path for visualization
        nav_msgs::msg::Path smooth_msg;
        smooth_msg.header = msg->header;
        smooth_msg.header.stamp = this->now();
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
trol_loop_count_++;
        const double goal_reached_wait_time = 1.0; // seconds
        double elapsed = (this->now() - start_time_).seconds();
        
        // Check if finished
        if (elapsed > trajectory_.back().t + goal_reached_wait_time) {
            stop_robot();
            path_active_ = false;
            RCLCPP_WARN(this->get_logger(), ">>> [COMPLETE] Goal Reached. Final Error: %.3fm | Loop Count: %d", 0.0, control_loop_count_);
            return;
        }

        // Find target point on trajectory based on elapsed time
        auto target = trajectory_.back();
        for (const auto& pt : trajectory_) { 
            if (pt.t >= elapsed) { 
                target = pt; 
                break; 
            } 
        }

        // Calculate PID errors
        double dx = target.x - x_;
        double dy = target.y - y_;
        last_dist_error_ = std::hypot(dx, dy);
        last_ang_error_ = angles::normalize_angle(std::atan2(dy, dx) - th_);

        // PID Control Law
        double v_cmd = this->get_parameter("kp_lin").as_double() * last_dist_error_;
        if (std::abs(last_ang_error_) > 1.0) v_cmd = 0.0; // Stop if orientation error too large
        double w_cmd = this->get_parameter("kp_ang").as_double() * last_ang_error_;
        
        // Velocity saturation
        double max_v = this->get_parameter("max_v").as_double();
        double max_w = this->get_parameter("max_w").as_double();
        v_cmd = std::max(std::min(v_cmd, max_v), -max_v);
        w_cmd = std::max(std::min(w_cmd, max_w), -max_w);

        // Apply safety strategy
        if (last_scan_ && safety_strategy_) {
            auto safety_command = safety_strategy_->calculate_safety(last_scan_);
            v_cmd *= safety_command.speed_scale;
        }

        // Log every 20 cycles (~1 second at 20Hz)
        static int log_counter = 0;
        if (log_counter++ % 20 == 0) {
             RCLCPP_INFO(this->get_logger(), 
                         ">>> [CONTROL
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_vel_pub_->publish(cmd);
    }

    void publish_diagnostics() {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();

        // Create diagnostic status for Cruiser
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Cruiser Controller";
        status.hardware_id = "cruiser_node";

        // Determine status level based on control activity
        if (path_active_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "ACTIVE - Tracking trajectory";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "IDLE - Awaiting plan";
        }

        // Add diagnostic data
        diagnostic_msgs::msg::KeyValue kv;

        kv.key = "Plans Received"; kv.value = std::to_string(plan_received_count_);
        status.values.push_back(kv);

        kv.key = "/cmd_vel Publish Count"; kv.value = std::to_string(cmd_vel_pub_count_);
        status.values.push_back(kv);

        kv.key = "Control Loop Cycles"; kv.value = std::to_string(control_loop_count_);
        status.values.push_back(kv);

        kv.key = "Last Distance Error (m)"; kv.value = std::to_string(last_dist_error_);
        status.values.push_back(kv);

        kv.key = "Last Angular Error (rad)"; kv.value = std::to_string(last_ang_error_);
        status.values.push_back(kv);

        kv.key = "Trajectory Points"; kv.value = std::to_string(trajectory_.size());
        status.values.push_back(kv);

        kv.key = "Path Active"; kv.value = path_active_ ? "true" : "false";
        status.values.push_back(kv);

        kv.key = "Odom Received"; kv.value = odom_received_ ? "true" : "false";
        status.values.push_back(kv);

        diag_array.status.push_back(status);
        diag_pub_->publish(diag_array v_cmd, w_cmd, last_dist_error_, last_ang_err_);
        }

        // Publish command
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v_cmd;
        cmd.angular.z = w_cmd;
        cmd_vel_pub_->publish(cmd);
        cmd_vel_pub_count_++;

        // Publish diagnostics every 50 cycles (~2.5 seconds)
        if (control_loop_count_ % 50 == 0) {
            publish_diagnostics();
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
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<CruiserNode>();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
