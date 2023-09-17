#include "planner.hpp"

Planner::Planner()
    : Node("Planner")
{   

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("joint_topic", "/JointState");
    this->declare_parameter("pose_topic", "/MocapPose");
    this->declare_parameter("pub_topic", "/ref_pose");

    /* Actually get all the parameters */
    this->_frequency =  this->get_parameter("frequency").as_double();

    /* Init Timer and subscribers*/
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                            std::bind(&Planner::_timer_callback, this));
    this->_joint_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        this->get_parameter("joint_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_joint_callback, this, std::placeholders::_1));
    this->_mocap_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pose_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_mocap_callback, this, std::placeholders::_1));
    
    /* Init Publishers */
    this->_setpoint_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pub_topic").as_string(), 10);

    /* Remember the beginning time stamp */
    this->_beginning = this->now();

}

/* Callback Functions */
void Planner::_timer_callback()
{
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "World";
    msg.header.stamp = this->now();
    msg.pose = this->get_trajectory_setpoint();
    this->_setpoint_publisher->publish(msg);
}


void Planner::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->curr_js = *msg;
}

void Planner::_mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->curr_pos = *msg;
}
