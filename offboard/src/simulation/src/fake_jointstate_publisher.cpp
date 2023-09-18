#include "fake_jointstate_publisher.hpp"
#include "common/common.hpp"

#include <Eigen/Dense>

FakeJointStatePublisher::FakeJointStatePublisher(): 
    Node("fakeJointStatePublisher"), _position(3)
{
    using namespace std::chrono_literals;

    this->declare_parameter("frequency", 25.0);
    this->declare_parameter("wall_yaw", 0.0);
    this->declare_parameter("sub_topic", "/MocapPose");
    this->declare_parameter("pub_topic", "/JointState");

    this->_wall_yaw = this->get_parameter("wall_yaw").as_double();

    /* Init Timer and subscribers*/
    this->_timer = this->create_wall_timer(
        1.0 / this->get_parameter("frequency").as_double() * 1s,
        std::bind(&FakeJointStatePublisher::_timer_callback, this));

    this->_vehicle_odom_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("sub_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&FakeJointStatePublisher::_handle_visual_odometry, this, std::placeholders::_1));
    
    /* Init Publishers */
    this->_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        this->get_parameter("pub_topic").as_string(), 10);
    

}

void FakeJointStatePublisher::_timer_callback()
{
    sensor_msgs::msg::JointState msg{};
    msg.header.stamp = this->now();
    msg.position = {0.0, 0.0};

    if(this->_position.at(1) > 1.0)
    {
        msg.position[0] = -0.02;
        msg.position[1] = this->_base_yaw - this->_wall_yaw;
    }

    this->_joint_state_pub->publish(msg);
}


void FakeJointStatePublisher::_handle_visual_odometry(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->_position.at(0) = msg->pose.position.x;
    this->_position.at(1) = msg->pose.position.y;
    this->_position.at(2) = msg->pose.position.z;

    Eigen::Quaterniond q(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
    this->_base_yaw = common::yaw_from_quaternion(q);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<FakeJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
