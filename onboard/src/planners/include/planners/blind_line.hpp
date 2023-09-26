#include "rclcpp/rclcpp.hpp"
#include "blind_planner.hpp"

class BlindLine : public BlindPlanner
{
public:
    BlindLine();
    /**
     * @brief Publish a trajectory setpoint
     */
    Eigen::Vector3d get_trajectory_setpoint() override;
    //   void align_to_wall(Eigen::Matrix3d *R_IB,Eigen::Vector3d *pos_IB,Eigen::Vector3d pos_IE, float32 encoderYaw, float32 mocapYaw, Eigen::Vector3d pos_BE) override;
};
