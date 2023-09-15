#include "test_trajectory.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

/**
 * @brief Align UAV to be perpendicular to wall (i.e. encoderYaw == 0). The function is designed to run permanentely after the trajectory position has been set
 * Inputs:
 *      pos_IE: position of End-Effector-Tip in World-Frame 
        encoderYaw: reading of encoder (in rad)
        mocapYaw: current yaw angle of UAV
        pos_BE:  position offset from UAV to End-Effector
* Outputs:
*   pos_IB_new: updated position of UAV
*   R_IB_new: updated orientation of UAV
 */
void TestTrajectoryPublisher::align_to_wall(float32[3] pos_IE, float32 encoderYaw, float32 mocapYaw,float32[3] pos_BE)
{
    // todo return orientation instead of void

    const float vel = 0.05; // yaw velocity in rad/s
    const float encoderYaw_round = round(encoderYaw);
    float increment = copysign(vel,encoderYaw_round);

    // limit increment to prevent overshoot
    if (abs(increment) > abs(encoderYaw))
    {
        increment = encoderYaw;
    }

    const float yaw = moCapYaw+increment;

    // return position and orientation of uav
    pos_IB_new = pos_IE-RotZ(yaw)*pos_BE;
    R_IB_new = RotZ(yaw);
    

}
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
