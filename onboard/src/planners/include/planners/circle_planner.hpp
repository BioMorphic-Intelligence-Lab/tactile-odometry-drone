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
    geometry_msgs::msg::Pose get_trajectory_setpoint() override;

private:
    double _L_x, _L_z, _v_x, _v_z;
};
