#include "circle_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

CirclePlanner::CirclePlanner()
{
    this->declare_parameter("d", 0.5);  // diameter of circle
    this->declare_parameter("T", 15.0); // duration for circle
    this->_d = this->get_parameter("d").as_double();
    this->_T = this->get_parameter("T").as_double();
}

/**
 * @brief Publish a trajectory setpoint.
 *
 generate circle, starting at 12 o'clock, roatating clockwise
    */
Eigen::Vector3d CirclePlanner::get_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (time > 0)
    {
        const float angle = 2 * M_PI / this->_T * time / this->_T;

        position.x() = 0.5 * this->_d * sin(angle);
        position.z() = 0.5 * this->_d * (cos(angle) - 1);
    }

    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CirclePlanner>());
    rclcpp::shutdown();
    return 0;
}
