#ifndef THE_CRUISER__TYPES__TRAJECTORY_POINT_HPP_
#define THE_CRUISER__TYPES__TRAJECTORY_POINT_HPP_

namespace the_cruiser {
namespace types {

struct TrajectoryPoint {
    double x;
    double y;
    double t; // Time
    double v; // Velocity (Optional, but useful for debugging)
};

} // namespace types
} // namespace the_cruiser
#endif
