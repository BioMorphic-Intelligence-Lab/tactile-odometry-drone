#ifndef PLANNER_H
#define PLANNER_H

#include <Eigen/Dense>

#include "common/common.hpp"
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
    virtual std::vector<double> get_trajectory_setpoint() = 0;
    
    rclcpp::Time _beginning;
    
private: 

    const double JS_THRESHOLD;
    double _frequency, _yaw_rate;
    bool _align, _in_contact;

    Eigen::Vector3d _ee_offset, _start_point;

    sensor_msgs::msg::JointState _curr_js;
    geometry_msgs::msg::PoseStamped _curr_pos;


    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _mocap_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _setpoint_publisher;
    
    /* Callback Functions */
    void _timer_callback();
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
    void align_to_wall(float &yaw_IB, Eigen::Vector3d &pos_IB,
                    Eigen::Vector3d pos_IE,
                    Eigen::Vector3d pos_BE,
                    float encoder_yaw=0,
                    float mocap_yaw=0);

};

#endif //PLANNER_H

