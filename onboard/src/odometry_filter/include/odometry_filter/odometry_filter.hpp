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
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class OdometryFilter : public rclcpp::Node
{
public:
    OdometryFilter();
    Eigen::Vector3d odom_to_wall(Eigen::Vector3d pos_odom, Eigen::Quaterniond quat_imu);
    Eigen::Vector3d wall_to_world(Eigen::Vector3d pos_wall);
    void evaluate_contact();
    void _detect_contact();

    Eigen::Matrix3d R_EO = Eigen::Matrix3d::Identity(),
                    R_WE_0 = Eigen::Matrix3d::Identity(), R_IB = Eigen::Matrix3d::Identity(), R_BW = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond quat_imu = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Quaterniond quat_moCap = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Quaterniond orientation_at_contact = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Vector3d pos_odom_last = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d pos_odom_at_init = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d pos_wall_last = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d _trackball_pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d pos_at_contact = Eigen::Vector3d(0, 0, 0);                  // freezed value of pos_ekf2 at time of contact
    Eigen::Vector3d pos_ekf2 = Eigen::Vector3d(0, 0, 0);                        // position from EKF2 filter on PX4
    Eigen::Vector3d ee_to_ball_offset = Eigen::Vector3d(-0.01875, 0.0423, 0.0); // in Frame E, located at the center of rotational axis
    Eigen::Vector3d ee_to_base_offset = Eigen::Vector3d(-0.03, 0.33, 0.046);

    bool in_contact = false;
    bool in_contact_last = false;
    bool is_init = false;

    double encoder_yaw = 0.0, encoder_yaw_at_contact = 0.0, yaw_at_contact = 0.0;
    double linear_joint = 0.0; // poti read in m

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroService;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_publisher_, odom_pose_wall_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _roll_publisher_, _yaw_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _trackball_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _contact_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _state_subscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;

    void _trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void _state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void _imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void _contact_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

#endif // ODOMETRY_H
