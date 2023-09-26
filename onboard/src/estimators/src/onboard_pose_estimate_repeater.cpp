#include "onboard_pose_estimate_repeater.hpp"

#include <Eigen/Dense>
#include "common/common.hpp"

using namespace personal;

OnboardPoseEstimateRepeater::OnboardPoseEstimateRepeater():Node("OnboardPoseEstimateRepeater")
{
    this->_pose_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",
                rclcpp::SensorDataQoS(),
                std::bind(
                        &OnboardPoseEstimateRepeater::_pose_callback,
                        this,
                        std::placeholders::_1
                    )
            );
    this->_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/onboard/pose_estimate", 10);
}


void OnboardPoseEstimateRepeater::_pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "world";

    Eigen::Vector3d ned_p(msg->position[0],msg->position[1],msg->position[2]);
    Eigen::Quaterniond ned_q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);    

    Eigen::Vector3d enu_p = common::ned_2_enu(ned_p);
    Eigen::Quaterniond enu_q = common::ned_2_enu(ned_q);

    pose.pose.position.x = enu_p.x();
    pose.pose.position.y = enu_p.y();
    pose.pose.position.z = enu_p.z();

    pose.pose.orientation.w = enu_q.w();
    pose.pose.orientation.x = enu_q.x();
    pose.pose.orientation.y = enu_q.y();
    pose.pose.orientation.z = enu_q.z();

    this->_pose_publisher->publish(pose);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<OnboardPoseEstimateRepeater>());
    rclcpp::shutdown();
    return 0;
}
