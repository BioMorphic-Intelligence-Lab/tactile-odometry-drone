#include "test_trajectory.hpp"
#include <Eigen/Dense>

/**
 * @brief Align UAV to be perpendicular to wall (i.e. encoderYaw == 0). The function is designed to run permanentely after the trajectory position has been set
 * Inputs:
 *      pos_IE: position of End-Effector-Tip in World-Frame 
        encoderYaw: reading of encoder (in rad)
        mocapYaw: current yaw angle of UAV
        pos_BE:  position offset from UAV to End-Effector
* Outputs:
*   pos_IB: updated position of UAV
*   R_IB: updated orientation of UAV
 
void TestTrajectoryPublisher::align_to_wall(Eigen::Matrix3d *R_IB,Eigen::Vector3d *pos_IB,Eigen::Vector3d pos_IE, float32 encoderYaw, float32 mocapYaw, Eigen::Vector3d pos_BE)
{
    
    const float velthis->get_parameter("yaw_rate").as_float();
    const float encoderYaw_round = round(encoderYaw);
    float increment = copysign(vel,encoderYaw_round);

    // limit increment to prevent overshoot
    if (abs(increment) > abs(encoderYaw))
    {
        increment = encoderYaw;
    }

    const float yaw = moCapYaw+increment;

    // return position and orientation of uav
    pos_IB = pos_IE-common::rot_z(yaw)*pos_BE;
    R_IB= common::rot_z(yaw);
    

}*/
/**
 * @brief Publish a trajectory setpoint.
 */
geometry_msgs::msg::Pose TestTrajectoryPublisher::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    geometry_msgs::msg::Pose msg{};

    if(t < 20)
    {
        msg.position.x = 0;
        msg.position.y = -0.90;
        msg.position.z = 1.75;
        
        if(fabs((int)t - t) < 0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Mission start. T-%f s.", 20.0 - t);
        } 
    }
    /* After mission time ran out */
    else if(t >= 20 && t < 25)
    {
       
        msg.position.x = 1.97;
        msg.position.y = -0.90;
        msg.position.z = 1.75;
    }
    else
    {

        msg.position.x = 1.97;
        msg.position.y = -0.90f + (t-25)*0.1f;
        msg.position.z = 1.75;
    }

    if(fabs((int)t - t) < 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Current Reference: [%f, %f, %f]",
            msg.position.x, msg.position.y, msg.position.z);
    }

    return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TestTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
