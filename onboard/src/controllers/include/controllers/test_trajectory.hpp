#include "rclcpp/rclcpp.hpp"
#include "controller.hpp"

class TestTrajectoryPublisher : public Controller
{
public:
    TestTrajectoryPublisher();
    /**
     * @brief Publish a trajectory setpoint
     */
    geometry_msgs::msg::Pose get_trajectory_setpoint() override;
 //   void align_to_wall(Eigen::Matrix3d *R_IB,Eigen::Vector3d *pos_IB,Eigen::Vector3d pos_IE, float32 encoderYaw, float32 mocapYaw, Eigen::Vector3d pos_BE) override;

};
