#include "planner.hpp"

class RectanglePlanner : public Planner
{
public:
    RectanglePlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    std::vector<double> get_trajectory_setpoint() override;

private:
    double _L_x, _L_z, _v_x, _v_z;
};
