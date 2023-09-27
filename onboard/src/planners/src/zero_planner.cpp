#include "zero_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

ZeroPlanner::ZeroPlanner()
{
}

Eigen::Vector3d ZeroPlanner::get_trajectory_setpoint()
{
    Eigen::Vector3d position(0, 0, 0);

    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ZeroPlanner>());
    rclcpp::shutdown();
    return 0;
}
