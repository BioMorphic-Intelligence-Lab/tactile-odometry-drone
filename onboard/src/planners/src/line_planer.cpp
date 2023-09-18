#include "line_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

LinePlanner::LinePlanner()
{
    this->declare_parameter("L_x", 1.0); // end position in m
    this->declare_parameter("L_z", 0.0); // end position in m
    this->declare_parameter("v", 0.1);   // velocity in m/s
    this->_L_x = this->get_parameter("L_x").as_double();
    this->_L_z = this->get_parameter("L_z").as_double();
    this->_v = this->get_parameter("v").as_double();

    /**
     * @brief Publish a trajectory setpoint.
     *
     generate line, starting at 12 o'clock, roatating clockwise
     */
    geometry_msgs::msg::Pose LinePlanner::get_trajectory_setpoint()
    {
        float time = (this->now() - this->_beginning).seconds();
        geometry_msgs::msg::Pose msg;

        const float angle = atan2(this->_L_z, this->_L_x);
        const float L = sqrt(this->_L_x ^ 2 + this->_L_z ^ 2);
        const float T = L / this->_v;

        const float v_x = this->_v * cos(angle);
        const float v_z = this->_v * sin(angle);

        if (time < 0)
        {
            time = 0;
        }
        if (time > T)
        {
            time = T;
        }
        float p_x = v_x * time;
        float p_z = v_z * time;

        msg.position.x = p_x;
        msg.position.y = 0;
        msg.position.z = p_z;
        return msg;
    }

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);

        rclcpp::spin(std::make_shared<LinePlanner>());
        rclcpp::shutdown();
        return 0;
    }
