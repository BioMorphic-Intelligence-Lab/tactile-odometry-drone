#include "rclcpp/rclcpp.hpp"
#include "common/common.hpp"
#include "planner.hpp"
#include <math.h>

class SinPlanner : public Planner
{
public:
    SinPlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    Eigen::Vector3d get_trajectory_setpoint() override;

private:
    double _L_x, _h_z, _v_x, _N_p;
};
