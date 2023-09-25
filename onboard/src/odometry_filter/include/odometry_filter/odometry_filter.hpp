#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <chrono>
#include <memory>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "common/common.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class OdometryFilter : public rclcpp::Node
{
public:
    OdometryFilter();
    Eigen::Vector3d odom_to_wall(Eigen::Vector3d pos_odom, Eigen::Quaterniond quat_imu);

    Eigen::Matrix3d R_EO, R_WE_0;
    Eigen::Quaterniond quat_imu; //(1, 0, 0, 0);
    Eigen::Vector3d pos_odom_last, pos_wall_last, _trackball_pos;

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroService;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _roll_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _trackball_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;

    void _trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void _imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif // ODOMETRY_H
