#ifndef PLANNER_H
#define PLANNER_H

#include <Eigen/Dense>

#include "common/common.hpp"
#include "std_msgs_stamped/msg/bool_stamped.hpp"
#include "std_msgs_stamped/msg/float64_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

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
    double _linear_axis_error_integral = 0.0;
    bool _align, _in_contact, _in_contact_old, _is_aligned, _contact_temp = false;

    std::vector<Eigen::Vector3d> _ee_offsets;

    Eigen::Quaterniond _quat_IB_des_old = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0); // yaw 180°
    Eigen::Quaterniond _quat_IB_at_contact, _quat_IO_at_contact;

    Eigen::Vector3d _ee_offset, _start_point, _trackball_pos, _current_position, _current_ee_position, _pos_IO_at_contact;
    Eigen::Quaterniond _current_quat, _current_ee_quat;

    sensor_msgs::msg::JointState _curr_js, _last_js;

    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _mocap_subscription, _ee_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _trackball_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _setpoint_publisher, _setpoint_publisher_ee, _contact_pose_publisher;
    rclcpp::Publisher<std_msgs_stamped::msg::Float64Stamped>::SharedPtr _force_publisher;
    rclcpp::Publisher<std_msgs_stamped::msg::BoolStamped>::SharedPtr _contact_publisher, _aligned_publisher;

    /* TF publisher */
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    /* Callback Functions */
    void _timer_callback();
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void _trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void _ee_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    geometry_msgs::msg::Pose eigen_pose_to_geometry_pose(Eigen::Vector3d position, Eigen::Quaterniond quat);

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
    void _align_to_wall(Eigen::Quaterniond quat_IO_at_contact, Eigen::Quaterniond quat_IB_des_old, Eigen::Vector3d pos_IO_des_0, Eigen::Vector3d pos_WO, float encoder_yaw, Eigen::Quaterniond &quat_IB_des_new, Eigen::Vector3d &pos_IB_des, Eigen::Matrix3d &R_IW);

    double _control_contact_force(float linear_joint, float desired_joint);

    void _get_uav_to_ee_position();

    bool _detect_contact();

    void _publish_forward_kinematics(Eigen::Matrix3d R_IB,
                                     Eigen::Vector3d p_IB,
                                     double joint_state[2]);

    void _publish_tf(Eigen::Matrix3d R,
                     Eigen::Vector3d p,
                     std::string child_name);

    geometry_msgs::msg::Transform _transform_from_eigen(Eigen::Quaterniond rot, Eigen::Vector3d pos);
};

#endif // PLANNER_H
