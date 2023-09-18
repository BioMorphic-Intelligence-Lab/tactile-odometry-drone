#include "sin_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

SinPlanner::SinPlanner()
{
    this->declare_parameter("L_x", 1.0); // end position in m
    this->declare_parameter("h_z", 0.0); // aplitude of sin in m
    this->declare_parameter("v_x", 0.1); // velocity in m/s
    this->declare_parameter("N_p", 2);   // number of periods per L_x
    this->_L_x = this->get_parameter("L_x").as_double();
    this->_h_z = this->get_parameter("h_z").as_double();
    this->_v = this->get_parameter("v").as_double();
    this->_N_p = this->get_parameter("N_p").as_double();

    /**
     * @brief Publish a trajectory setpoint.
     *
     generate line in x-direction with sin-function in z-direction
     */
    geometry_msgs::msg::Pose SinPlanner::get_trajectory_setpoint()
    {
        float time = (this->now() - this->_beginning).seconds();
        geometry_msgs::msg::Pose msg;

        const float T = _L_x / _v_x; // duration of line

        if (time < 0)
        {
            time = 0;
        }
        if (time > T)
        {
            time = T;
        }
        float p_x = v_x * time;
        float p_z = this->_h_z * sin(this->_N_p * 2 * M_PI / this->_L_x * p_x);
        ;

        msg.position.x = p_x;
        msg.position.y = 0;
        msg.position.z = p_z;
        return msg;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SinPlanner>());
    rclcpp::shutdown();
    return 0;
}
