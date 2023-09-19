#include "rectangle_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
Eigen::Vector3d RectanglePlanner::get_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

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

    position.x() = p_x;
    position.z() = p_z;
    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<RectanglePlanner>());
    rclcpp::shutdown();
    return 0;
}
