#include "rectangle_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
 */
void RectanglePlanner::align_to_wall(Eigen::Matrix3d *R_IB, Eigen::Vector3d *pos_IB, Eigen::Vector3d pos_IE, float encoderYaw, float mocapYaw, Eigen::Vector3d pos_BE)
{

    const float vel = this->get_parameter("yaw_rate").as_double();
    const float encoderYaw_round = round(encoderYaw);
    float increment = copysign(vel, encoderYaw_round);

    // limit increment to prevent overshoot
    if (abs(increment) > abs(encoderYaw))
    {
        increment = encoderYaw;
    }

    const float yaw = mocapYaw + increment;

    // return position and orientation of uav
    *pos_IB = (pos_IE) - common::rot_z(yaw) * (pos_BE);
    *R_IB = common::rot_z(yaw);
}

RectanglePlanner::RectanglePlanner()
{
    this->declare_parameter("L_x", 1.0);
    this->declare_parameter("L_z", 0.5);
    this->declare_parameter("v_x", 0.1);
    this->declare_parameter("v_z", 0.1);
    this->_L_x = this->get_parameter("L_x").as_double();
    this->_L_z = this->get_parameter("L_z").as_double();
    this->_v_x = this->get_parameter("v_x").as_double();
    this->_v_z = this->get_parameter("v_z").as_double();
}
/**
 * @brief Publish a trajectory setpoint.
 *
 1 ------- 2
 |         |
 |         |
 |         |
 4---------3
 */
geometry_msgs::msg::Pose RectanglePlanner::get_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    geometry_msgs::msg::Pose msg;

    double Tx = this->_L_x / this->_v_x;
    double Tz = this->_L_z / this->_v_z;
    double p_x = 0;
    double p_z = 0;
    if (time > 0)
    {
        if (time <= Tx) // line 1-2
        {
            p_x = time * this->_v_x;
            p_z = 0;
        }
        else if (time <= Tx + Tz) // line 2-3
        {
            p_z = -(time - Tx) * this->_v_z;
            p_x = this->_L_x;
        }
        else if (time <= 2 * Tx + Tz) // line 3-4
        {
            p_x = this->_L_x - (time - Tx - Tz) * this->_v_x;
            p_z = -this->_L_z;
        }
        else if (time <= 2 * Tx + 2 * Tz) // line 4-1
        {
            p_x = 0;
            p_z = -this->_L_z + (time - 2 * Tx - Tz) * this->_v_z;
        }
        else // time is greater than rectangle duration
        {
            p_x = 0;
            p_z = 0;
        }
    }
    
    msg.position.x = p_x;
    msg.position.y = 0;
    msg.position.z = p_z;
    return msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<RectanglePlanner>());
    rclcpp::shutdown();
    return 0;
}
