#include "blind_circle.hpp"
#include <Eigen/Dense>

BlindCircle::BlindCircle()
{
}

/**
 * @brief Publish a trajectory setpoint.
 */
Eigen::Vector3d BlindCircle::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    Eigen::Vector3d position;

    if (t < 20)
    {
        position.x() = 0.75;
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
        position.x() = 0.75;
        position.y() = this->_depth / this->_approach_time * (t - 20);
        position.z() = 1.7;
    }
    /* In Contact */
    else if (t >= 20.0 + this->_approach_time &&
             t < 20.0 + this->_approach_time + 5.0)
    {
        position.x() = 0.75;
        position.y() = this->_depth;
        position.z() = 1.7; 
    }
    /* Start Circle */
    else
    {
        double time = t -  (20.0 + this->_approach_time + 5.0);
        position.x() = 0.75 + 0.25 * sin(2*M_PI / 10.0 * time + M_PI) ;
        position.y() = this->_depth;
        position.z() = 1.7 + (0.25 * cos(2*M_PI / 10.0 * time + M_PI));
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

    rclcpp::spin(std::make_shared<BlindCircle>());
    rclcpp::shutdown();
    return 0;
}
