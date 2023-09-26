#include "blind_sine.hpp"
#include <Eigen/Dense>

BlindSine::BlindSine()
{
}

/**
 * @brief Publish a trajectory setpoint.
 */
Eigen::Vector3d BlindSine::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (t < 20)
    {
        position.x() = 0;
        position.y() = 0.0;
        position.z() = 1.7;

        if (fabs((int)t - t) < 0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Mission start. T-%.2f s.", 20.0 - t);
        }
    }
    /* Approach */
    else if (t >= 20.0 && t < 20.0 + this->_approach_time)
    {
        position.x() = 0.0;
        position.y() = this->_depth / this->_approach_time * (t - 20);
        position.z() = 1.7;
    }
    /* In Contact */
    else if (t >= 20.0 + this->_approach_time &&
             t < 20.0 + this->_approach_time + 5.0)
    {
        position.x() = 0.0;
        position.y() = this->_depth;
        position.z() = 1.7; 
    }
    /* Start Sine */
    else
    {
        double time = t -  (20.0 + this->_approach_time + 5.0);
        position.x() = time * 0.1f;
        position.y() = this->_depth;
        position.z() = 1.7 - (0.25 * sin(time));
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

    rclcpp::spin(std::make_shared<BlindSine>());
    rclcpp::shutdown();
    return 0;
}
