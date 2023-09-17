#include "rclcpp/rclcpp.hpp"
#include "common/common.hpp"
#include "planner.hpp"

class RectanglePlanner : public Planner
{
public:
    RectanglePlanner();
    /**
     * @brief Publish a trajectory setpoint
     */
    geometry_msgs::msg::Pose get_trajectory_setpoint() override;

    void align_to_wall(Eigen::Matrix3d *R_IB, Eigen::Vector3d *pos_IB, Eigen::Vector3d pos_IE, float encoderYaw, float mocapYaw, Eigen::Vector3d pos_BE);

private:
    double _L_x, _L_z, _v_x, _v_z;
};
