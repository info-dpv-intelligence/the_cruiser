#ifndef THE_CRUISER__INTERFACES__I_SAFETY_STRATEGY_HPP_
#define THE_CRUISER__INTERFACES__I_SAFETY_STRATEGY_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"

namespace the_cruiser {
namespace interfaces {

struct SafetyCommand {
    double speed_scale; // 0.0 (Stop) to 1.0 (Full Speed)
};

class ISafetyStrategy {
public:
    virtual ~ISafetyStrategy() = default;
    virtual SafetyCommand calculate_safety(const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

} // namespace interfaces
} // namespace the_cruiser
#endif
