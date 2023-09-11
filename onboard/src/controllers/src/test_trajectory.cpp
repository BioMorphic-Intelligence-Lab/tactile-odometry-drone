#include "test_trajectory.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

/**
 * @brief Publish a trajectory setpoint.
 */
void TestTrajectoryPublisher::publish_trajectory_setpoint()
{
    float t = (this->now() - this->_beginning).seconds();
    px4_msgs::msg::TrajectorySetpoint msg{};

    if(t < 20)
    {
        msg.position = {0, 0, -1.7};
        msg.yaw = 0.0;

        RCLCPP_INFO(this->get_logger(), "Mission start. T-%f s.", 20.0 - t);
    }
    /* After mission time ran out */
    else if(t >= 20 && t < 25)
    {
        msg.position = {-1.6, -0.10, -1.7};
        msg.yaw = 0.0;
    }
    else
    {
        msg.position = {-1.6, -0.1f + (t-25)*0.01f , -1.7};
        msg.yaw = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "Current Reference: [%f, %f, %f]",
        msg.position[0], msg.position[1], msg.position[2]);

    msg.timestamp = this->get_timestamp();
    this->_trajectory_publisher->publish(msg);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<TestTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
