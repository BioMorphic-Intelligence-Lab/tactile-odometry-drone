#include "test_trajectory.hpp"
#include <Eigen/Dense>

TestTrajectoryPublisher::TestTrajectoryPublisher()
{
}

/**
 * @brief Publish a trajectory setpoint.
 */
Eigen::Vector3d TestTrajectoryPublisher::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (t < 20)
    {
        position.x() = 0;
        position.y() = -0.90;
        position.z() = 1.85;

        if (fabs((int)t - t) < 0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Mission start. T-%f s.", 20.0 - t);
        }
    }
    /* After mission time ran out */
    else if (t >= 20 && t < 25)
    {
        position.x() = 1.97;
        position.y() = -0.90;
        position.z() = 1.85;
    }
    else
    {
        position.x() = 1.97;
        position.y() = -0.90f + (t - 25) * 0.1f;
        position.z() = 1.85;
    }

    if (fabs((int)t - t) < 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Current Reference: [%f, %f, %f]",
                    position.x(), position.y(), position.z());
    }

    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TestTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
