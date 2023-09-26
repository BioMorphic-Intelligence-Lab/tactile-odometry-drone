#include "blind_planner.hpp"

#include "rclcpp/time.hpp"

using namespace personal;
BlindPlanner::BlindPlanner()
    : Node("BlindPlanner")
{

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("pub_topic", "/ref_pose");

    this->_frequency = this->get_parameter("frequency").as_double();

    /* Init Timer and subscribers*/
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                           std::bind(&BlindPlanner::_timer_callback, this));
    /* Init Publishers */
    this->_setpoint_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pub_topic").as_string(), 10);
    
    /* Init the timestamp to some time in the future value until we establish contact */
    this->_beginning = this->now();
}

/* Callback Functions */
void BlindPlanner::_timer_callback()
{
    /* Init the Message and time stamp it */
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();

    /* Put in the position of the planner */
    Eigen::Vector3d position = this->get_trajectory_setpoint();

    /* Add position to reference message */
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();

    /* Assume zero yaw as reference */
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    

    /* Finally publish the message */
    this->_setpoint_publisher->publish(msg);
}
