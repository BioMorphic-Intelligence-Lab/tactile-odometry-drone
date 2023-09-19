#include "planner.hpp"

class RectanglePlanner : public Planner
{
public:
    RectanglePlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    Eigen::Vector3d get_trajectory_setpoint() override;

private:
    double _L_x, _L_z, _v_x, _v_z;
};
