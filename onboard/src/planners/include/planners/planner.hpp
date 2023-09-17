#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class Planner : public rclcpp::Node
{
public:
    Planner();

    /**
     * @brief Publish a trajectory setpoint
     */
    virtual geometry_msgs::msg::Pose get_trajectory_setpoint() = 0;
    
    rclcpp::Time _beginning;
    
protected:

    sensor_msgs::msg::JointState curr_js;
    geometry_msgs::msg::PoseStamped curr_pos;

private: 

    double _frequency;

    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _mocap_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _setpoint_publisher;
    
    /* Callback Functions */
    void _timer_callback();
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

};


