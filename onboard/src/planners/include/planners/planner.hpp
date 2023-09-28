#ifndef PLANNER_H
#define PLANNER_H

#include <Eigen/Dense>

#include "common/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class Planner : public rclcpp::Node
{
public:
    Planner();

    /**
     * @brief Publish a trajectory setpoint
     */
    virtual Eigen::Vector3d get_trajectory_setpoint() = 0;

    rclcpp::Time _beginning;
    rclcpp::Time _time_of_first_contact;
    rclcpp::Time _approach_beginning;

private:
    const double JS_THRESHOLD;
    double _frequency, _yaw_rate;
    double _v_approach;                     // velocity with which to approach the start position
    double _alignment_threshold;            // angle threshold in rad defining is_aligned
    double _desired_linear_joint_pos;       // desired value for linear joint in m
    double _position_offset = 0.0;          // positon offset calculated by force controller in m
    double _minimum_contact_duration = 3.0; // minimum duration of contact befor contact is enabled
    bool _align, _in_contact, _is_aligned, _contact_temp;

    std::vector<Eigen::Vector3d> _ee_offsets;

    Eigen::Quaterniond output_q;

    Eigen::Vector3d _ee_offset, _start_point, _trackball_pos;

    sensor_msgs::msg::JointState _curr_js, _last_js;
    geometry_msgs::msg::PoseStamped _curr_pose, _curr_ee_pose;

    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _mocap_subscription, _ee_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _trackball_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _setpoint_publisher, _setpoint_publisher_ee;

    /* Callback Functions */
    void _timer_callback();
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void _trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void _ee_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Align UAV to be perpendicular to wall (i.e. encoderYaw == 0). The function is designed to run permanentely after the trajectory position has been set
     * Inputs:
     *      pos_IE: previously desired position of End-Effector-Tip in World-Frame
            encoderYaw: reading of encoder (in rad)
            mocapYaw: current yaw angle of UAV
            pos_BE:  position offset from UAV to End-Effector
    * Outputs:
    *   pos_IB: updated position of UAV
    *   yaw_IB: updated yaw of UAV
    */
    void _align_to_wall(Eigen::Quaterniond &quat_IB, Eigen::Vector3d &pos_IB, Eigen::Vector3d pos_WE, Eigen::Vector3d pos_BE, float encoder_yaw, Eigen::Quaterniond quat_mocap);

    double _control_contact_force(float linear_joint, float desired_joint);

    void _get_uav_to_ee_position();

    bool _detect_contact();
};

#endif // PLANNER_H
