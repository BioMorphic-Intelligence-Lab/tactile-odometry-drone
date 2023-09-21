#include "line_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

LinePlanner::LinePlanner()
{
    this->declare_parameter("L_x", 1.5); // end position in m
    this->declare_parameter("L_z", 0.0); // end position in m
    this->declare_parameter("v", 0.1);   // velocity in m/s
    this->_L_x = this->get_parameter("L_x").as_double();
    this->_L_z = this->get_parameter("L_z").as_double();
    this->_v = this->get_parameter("v").as_double();
}

/**
 * @brief Publish a trajectory setpoint.
 *
 generate line
    */
Eigen::Vector3d LinePlanner::get_trajectory_setpoint()
{
    double time = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d ref_position;
    ref_position << 0, 0, 0;
    
    if (time > 0.0)
    {
        const float angle = atan2(this->_L_z, this->_L_x);

        const float v_x = this->_v * cos(angle);
        const float v_z = this->_v * sin(angle);

        ref_position.x() = v_x * time;
        ref_position.z() = v_z * time;

    }

    return ref_position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LinePlanner>());
    rclcpp::shutdown();
    return 0;
}
