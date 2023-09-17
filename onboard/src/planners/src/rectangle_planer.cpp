#include "rectangle_planer.hpp"
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
void TestTrajectoryPublisher::align_to_wall(Eigen::Matrix3d *R_IB, Eigen::Vector3d *pos_IB, Eigen::Vector3d pos_IE, float32 encoderYaw, float32 mocapYaw, Eigen::Vector3d pos_BE)
{

    const float velthis->get_parameter("yaw_rate").as_float();
    const float encoderYaw_round = round(encoderYaw);
    float increment = copysign(vel, encoderYaw_round);

    // limit increment to prevent overshoot
    if (abs(increment) > abs(encoderYaw))
    {
        increment = encoderYaw;
    }

    const float yaw = moCapYaw + increment;

    // return position and orientation of uav
    pos_IB = pos_IE - common::rot_z(yaw) * pos_BE;
    R_IB = common::rot_z(yaw);
}

TestTrajectoryPublisher::TestTrajectoryPublisher()
{
    this->declare_parameter("L_x", 1.0);
    this->declare_parameter("L_z", 0.5);
    this->declare_parameter("v_x", 0.1);
    this->declare_parameter("v_z", 0.1);
    _L_x = this->get_parameter("L_x").as_double();
    _L_z = this->get_parameter("L_z").as_double();
    _v_x = this->get_parameter("v_x").as_double();
    _v_z = this->get_parameter("v_z").as_double();
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
geometry_msgs::msg::PoseStamped TestTrajectoryPublisher::publish_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    geometry_msgs::msg::PoseStamped msg;

    double Tx = L_x / _v_x;
    double Tz = L_z / v_z;
    double p_x = 0;
    double p_z = 0;
    if (time > 0)
    {
        if (time <= Tx) // line 1-2
        {
            p_x = time * _v_x;
            p_z = 0;
        }
        else if (time <= Tx + Tz) // line 2-3
        {
            p_z = -(time - Tx) * _v_z;
            p_x = L_x;
        }
        else if (time <= 2 * Tx + Tz) // line 3-4
        {
            p_x = L_x - (time - Tx - Tz) * _v_x;
            p_z = -L_z;
        }
        else if (time <= 2 * Tx + 2 * Tz) // line 4-1
        {
            p_x = 0;
            p_z = -L_z + (time - 2 * Tx - Tz) * _v_z;
        }
        else // time is greater than rectangle duration
        {
            p_x = 0;
            p_z = 0;
        }
    }
    msg.pose.position = {p_x, 0, p_z};
    msg.header.stamp = this->get_timestamp();
    return msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TestTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
