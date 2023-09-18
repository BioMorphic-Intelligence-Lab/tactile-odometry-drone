#include "test_trajectory.hpp"
#include <Eigen/Dense>

TestTrajectoryPublisher::TestTrajectoryPublisher()
{}

/**
 * @brief Publish a trajectory setpoint.
 */
std::vector<double> TestTrajectoryPublisher::get_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    std::vector<double> position(3);

    if(t < 20)
    {
        position.at(0) = 0;
        position.at(1) = -0.90;
        position.at(2) = 1.75;
        
        if(fabs((int)t - t) < 0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Mission start. T-%f s.", 20.0 - t);
        } 
    }
    /* After mission time ran out */
    else if(t >= 20 && t < 25)
    {
       
        position.at(0) = 1.97;
        position.at(1) = -0.90;
        position.at(2) = 1.75;
    }
    else
    {

        position.at(0) = 1.97;
        position.at(1) = -0.90f + (t-25)*0.1f;
        position.at(2) = 1.75;
    }

    if(fabs((int)t - t) < 0.05)
    {
        RCLCPP_INFO(this->get_logger(), "Current Reference: [%f, %f, %f]",
            position.at(0), position.at(1), position.at(2));
    }

    return position;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TestTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
