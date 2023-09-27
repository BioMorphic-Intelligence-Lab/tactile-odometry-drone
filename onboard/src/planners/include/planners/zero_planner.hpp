#include "planner.hpp"

class ZeroPlanner : public Planner
{
public:
    ZeroPlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    Eigen::Vector3d get_trajectory_setpoint() override;
};
