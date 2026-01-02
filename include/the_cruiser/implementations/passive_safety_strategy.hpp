#ifndef THE_CRUISER__IMPLEMENTATIONS__PASSIVE_SAFETY_STRATEGY_HPP_
#define THE_CRUISER__IMPLEMENTATIONS__PASSIVE_SAFETY_STRATEGY_HPP_

#include "the_cruiser/interfaces/i_safety_strategy.hpp"

namespace the_cruiser {
namespace implementations {

class PassiveSafetyStrategy : public interfaces::ISafetyStrategy {
public:
    interfaces::SafetyCommand calculate_safety(const sensor_msgs::msg::LaserScan::SharedPtr /*scan*/) override {
        return {1.0}; // Always 100% speed
    }
};

}
}
#endif
