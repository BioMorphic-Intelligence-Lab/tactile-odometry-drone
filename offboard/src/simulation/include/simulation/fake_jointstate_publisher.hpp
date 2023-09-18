#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class FakeJointStatePublisher : public rclcpp::Node
{
public:
    FakeJointStatePublisher();

private:

    /* Publishers and Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _vehicle_odom_sub;    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_pub;

    /* Timer */
    rclcpp::TimerBase::SharedPtr _timer;

    /* Callback functions */
    void _handle_visual_odometry(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void _timer_callback();

    /* Curr State */
    double _base_yaw, _wall_yaw;
    std::vector<double> _position;

};