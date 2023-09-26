#include "blind_line.hpp"
#include <Eigen/Dense>

BlindLine::BlindLine()
{
}

/**
 * @brief Publish a trajectory setpoint.
 */
Eigen::Vector3d BlindLine::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (t < 20)
    {
        position.x() = 0;
        position.y() = 0.0;
        position.z() = 1.85;

        if (fabs((int)t - t) < 0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Mission start. T-%.2f s.", 20.0 - t);
        }
    }
    /* Approach */
    else if (t >= 20 && t < 25)
    {
        position.x() = 0.0;
        position.y() = 1.0/5.0 * (t - 20);
        position.z() = 1.85;
    }
    /* In Contact */
    else if (t >=25 && t < 30)
    {
        position.x() = 0.0;
        position.y() = 1.0;
        position.z() = 1.85; 
    }
    /* Start Sine */
    else
    {
        position.x() = (t - 30) * 0.1f;
        position.y() = 1.00;
        position.z() = 1.85;
    }
    
    if (fabs((int)t - t) < 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Current Reference: [%.2f, %.2f, %.2f]",
                    position.x(), position.y(), position.z());
    }

    return position;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<BlindLine>());
    rclcpp::shutdown();
    return 0;
}
