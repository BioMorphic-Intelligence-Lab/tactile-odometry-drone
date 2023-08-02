#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class ForceEstimatorNode : public rclcpp::Node
{
public:
  ForceEstimatorNode();

private:
  const std::string _NAMES[2];

  void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr _force_publisher;

  double _k_lin, _k_rot, _d_ee;

};