#ifndef BLIND_PLANNER_H
#define BLIND_PLANNER_H

#include <Eigen/Dense>

#include "common/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using namespace std::chrono_literals;

class BlindPlanner : public rclcpp::Node
{
public:
    BlindPlanner();

    /**
     * @brief Publish a trajectory setpoint
     */
    virtual Eigen::Vector3d get_trajectory_setpoint() = 0;

    rclcpp::Time _beginning;

private:
    double _frequency;
    
    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _setpoint_publisher;

    /* Callback Functions */
    void _timer_callback();
};

#endif // BLIND_PLANNER_H
