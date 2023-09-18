#include "circle_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

CirclePlanner::CirclePlanner()
{
    this->declare_parameter("d", 1.0); // diameter of circle
    this->declare_parameter("T", 5);   // duration for circle
    this->_d = this->get_parameter("d").as_double();
    this->_T = this->get_parameter("T").as_double();

    /**
     * @brief Publish a trajectory setpoint.
     *
     generate circle, starting at 12 o'clock, roatating clockwise
     */
    geometry_msgs::msg::Pose CirclePlanner::get_trajectory_setpoint()
    {
        float time = (this->now() - this->_beginning).seconds();
        geometry_msgs::msg::Pose msg;

        const float angle = 2 * M_PI / T * (time % this->_T);
        float p_x, p_z = 0;
        if (time > 0)
        {
            p_x = this->_d / 2 * sin(angle);
            p_z = this->_d / 2 * cos(angle) - 1;
        }

        msg.position.x = p_x;
        msg.position.y = 0;
        msg.position.z = p_z;
        return msg;
    }

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);

        rclcpp::spin(std::make_shared<CirclePlanner>());
        rclcpp::shutdown();
        return 0;
    }
