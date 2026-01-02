#ifndef THE_CRUISER__INTERFACES__I_TRAJECTORY_GENERATOR_HPP_
#define THE_CRUISER__INTERFACES__I_TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "the_cruiser/types/trajectory_point.hpp"

namespace the_cruiser {
namespace interfaces {

class ITrajectoryGenerator {
public:
    virtual ~ITrajectoryGenerator() = default;
    virtual std::vector<types::TrajectoryPoint> smooth(const std::vector<geometry_msgs::msg::Point>& waypoints) = 0;
};

} // namespace interfaces
} // namespace the_cruiser
#endif
