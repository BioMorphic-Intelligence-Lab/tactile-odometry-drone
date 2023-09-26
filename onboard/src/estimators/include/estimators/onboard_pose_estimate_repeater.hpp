#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OnboardPoseEstimateRepeater : public rclcpp::Node
{
public:
  OnboardPoseEstimateRepeater();

private:
  
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _pose_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_publisher;

  void _pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

};