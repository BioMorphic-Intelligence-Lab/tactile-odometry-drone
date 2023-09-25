#include <chrono>
#include <memory>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class OdometryFilter : public rclcpp::Node
{
public:
  OdometryFilter()
      : Node("odometry_filter")
  {
    printf("constructor...");

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("trackball_position", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&OdometryFilter::timer_callback, this), timer_cb_group_);

    zeroService =
        this->create_service<std_srvs::srv::Trigger>("odometry_filter/setZero", std::bind(&OdometryFilter::setToZeroCB, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
                                                     service_cb_group_);
    printf("starting...");
  }

  ~OdometryFilter()
  {
    printf("destructor");
  }

private:
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  void timer_callback()
  {
  }
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroService;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tick_publisher_;

  void setToZeroCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::cout << "setZero service" << std::endl;
    response->success = true;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<OdometryFilter>();
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  return 0;
}
