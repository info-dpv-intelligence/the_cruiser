#include "the_cruiser/implementations/catmull_rom_generator.hpp"

namespace the_cruiser {
namespace implementations {

// CONSTRUCTOR - TUNING VALUES
CatmullRomGenerator::CatmullRomGenerator() 
    : step_size_(0.2), max_vel_(1.0), max_acc_(0.5) {}

std::vector<types::TrajectoryPoint> CatmullRomGenerator::smooth(const std::vector<geometry_msgs::msg::Point>& waypoints) {
    if (waypoints.size() < 4) return {};

    std::vector<geometry_msgs::msg::Point> dense_path;
    std::vector<geometry_msgs::msg::Point> padded_path = waypoints;
    // Handle endpoint padding
    padded_path.insert(padded_path.begin(), waypoints.front());
    padded_path.push_back(waypoints.back());

    // 1. Generate Spline
    for (size_t i = 1; i < padded_path.size() - 2; ++i) {
        auto p0 = padded_path[i - 1];
        auto p1 = padded_path[i];
        auto p2 = padded_path[i + 1];
        auto p3 = padded_path[i + 2];
        dense_path.push_back(p1);
        for (double t = step_size_; t < 1.0; t += step_size_) {
            dense_path.push_back(get_catmull_rom_point(t, p0, p1, p2, p3));
        }
    }
    dense_path.push_back(waypoints.back());

    // 2. Apply Physics Profile
    return apply_trapezoidal_profile(dense_path);
}

geometry_msgs::msg::Point CatmullRomGenerator::get_catmull_rom_point(double t, 
    const geometry_msgs::msg::Point& p0, const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& p3) 
{
    double t2 = t * t;
    double t3 = t2 * t;
    geometry_msgs::msg::Point res;
    
    res.x = 0.5 * ((2.0 * p1.x) + (-p0.x + p2.x) * t + 
        (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 + 
        (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
    res.y = 0.5 * ((2.0 * p1.y) + (-p0.y + p2.y) * t + 
        (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 + 
        (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
    return res;
}

std::vector<types::TrajectoryPoint> CatmullRomGenerator::apply_trapezoidal_profile(const std::vector<geometry_msgs::msg::Point>& path) {
    if (path.empty()) return {};

    // 1. Forward Pass (Acceleration Limit)
    std::vector<double> fwd = apply_pass(path);
    
    // 2. Backward Pass (Deceleration Limit)
    std::vector<geometry_msgs::msg::Point> rev_path = path;
    std::reverse(rev_path.begin(), rev_path.end());
    std::vector<double> bwd = apply_pass(rev_path);
    std::reverse(bwd.begin(), bwd.end()); // Reverse back to align with forward path

    // 3. Merge
    std::vector<types::TrajectoryPoint> traj;
    double current_time = 0.0;
    
    // Start Point (v=0)
    traj.push_back({path[0].x, path[0].y, current_time, 0.0});

    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Current velocity (at i)
        double v_curr = std::min(fwd[i], bwd[i]);
        // Next velocity (at i+1)
        double v_next = std::min(fwd[i+1], bwd[i+1]);

        double dist = std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
        
        // Average velocity for this segment
        double v_avg = std::max(0.001, (v_curr + v_next) / 2.0);
        
        current_time += dist / v_avg;

        traj.push_back({path[i+1].x, path[i+1].y, current_time, v_next});
    }
    return traj;
}

std::vector<double> CatmullRomGenerator::apply_pass(const std::vector<geometry_msgs::msg::Point>& path) {
    std::vector<double> vels(path.size(), 0.0);
    // Loop starts at 1 because v[0] is always 0
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
        double v_prev = vels[i-1];
        
        // Kinematics: vf^2 = vi^2 + 2*a*d
        double max_reach = std::sqrt(v_prev*v_prev + 2*max_acc_*dist);
        
        // Clip to global max limit
        vels[i] = std::min(max_reach, max_vel_);
    }
    return vels;
}

}
}
