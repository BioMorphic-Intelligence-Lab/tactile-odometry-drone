#include "circle_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

CirclePlanner::CirclePlanner()
{
    this->declare_parameter("d", 1.0); // diameter of circle
    this->declare_parameter("T", 15.0);   // duration for circle
    this->_d = this->get_parameter("d").as_double();
    this->_T = this->get_parameter("T").as_double();
}

/**
 * @brief Publish a trajectory setpoint.
 *
 generate circle, starting at 12 o'clock, roatating clockwise
    */
std::vector<double> CirclePlanner::get_trajectory_setpoint()
{
    float time = (this->now() - this->_beginning).seconds();
    std::vector<double> position(3);

    if (time > 0)
    {
        const float angle = 2 * M_PI / this->_T * fmod(time, this->_T);
     
        position.at(0) = 0.5 * this->_d * sin(angle);
        position.at(2) = 0.5 * this->_d * cos(angle) - 1;
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
