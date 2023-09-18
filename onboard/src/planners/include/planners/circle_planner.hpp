#include "rclcpp/rclcpp.hpp"
#include "common/common.hpp"
#include "planner.hpp"
#include <math.h>

class CirclePlanner : public Planner
{
public:
    CirclePlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    std::vector<double> get_trajectory_setpoint() override;

private:
    double _T, _d;
};
