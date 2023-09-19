#include "sin_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

SinPlanner::SinPlanner()
{
    this->declare_parameter("L_x", 1.0); // end position in m
    this->declare_parameter("h_z", 1.0); // amplitude of sin in m
    this->declare_parameter("v_x", 0.1); // velocity in m/s
    this->declare_parameter("N_p", 2.0); // number of periods per L_x
    this->_L_x = this->get_parameter("L_x").as_double();
    this->_h_z = this->get_parameter("h_z").as_double();
    this->_v_x = this->get_parameter("v_x").as_double();
    this->_N_p = this->get_parameter("N_p").as_double();
}

/**
 * @brief Publish a trajectory setpoint.
 * generate line in x-direction with sin-function in z-direction
 */
Eigen::Vector3d SinPlanner::get_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (time > 0)
    {
        float p_x = _v_x * time;
        position.x() = p_x;
        position.z() = this->_h_z * sin(this->_N_p * 2 * M_PI / this->_L_x * p_x);
    }

    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SinPlanner>());
    rclcpp::shutdown();
    return 0;
}
