#include "rclcpp/rclcpp.hpp"
#include "common/common.hpp"
#include "planner.hpp"
#include <math.h>

class LinePlanner : public Planner
{
public:
    LinePlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    Eigen::Vector3d get_trajectory_setpoint() override;

private:
    double _L_x, _L_z, _v;
};
