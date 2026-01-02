#ifndef THE_CRUISER__IMPLEMENTATIONS__CATMULL_ROM_GENERATOR_HPP_
#define THE_CRUISER__IMPLEMENTATIONS__CATMULL_ROM_GENERATOR_HPP_

#include "the_cruiser/interfaces/i_trajectory_generator.hpp"
#include <cmath>
#include <algorithm>

namespace the_cruiser {
namespace implementations {

class CatmullRomGenerator : public interfaces::ITrajectoryGenerator {
public:
    CatmullRomGenerator();
    std::vector<types::TrajectoryPoint> smooth(const std::vector<geometry_msgs::msg::Point>& waypoints) override;

private:
    // Tuning Parameters (Matches Python Prototype)
    double step_size_; 
    double max_vel_;
    double max_acc_;

    geometry_msgs::msg::Point get_catmull_rom_point(double t, 
                                                    const geometry_msgs::msg::Point& p0,
                                                    const geometry_msgs::msg::Point& p1,
                                                    const geometry_msgs::msg::Point& p2,
                                                    const geometry_msgs::msg::Point& p3);

    std::vector<types::TrajectoryPoint> apply_trapezoidal_profile(const std::vector<geometry_msgs::msg::Point>& path);
    std::vector<double> apply_pass(const std::vector<geometry_msgs::msg::Point>& path);
};

}
}
#endif
