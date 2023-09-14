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

class TrackballInterface : public rclcpp::Node
{
public:
  int fd = -1;
  int sum_x, sum_y = 0;
  double deltaT, deltaT_temp = 0.0;

  void read_mouse()
  {
    printf("read_mouse");
    int bytes;
    unsigned char data[3];
    int counter = 0;

    bool terminate_flag = 0;

    char *pDevice = "/dev/input/mouse0";
    std::cout << "trackball_name:" << trackball_name << " \n";
    if (!trackball_name.compare("X19"))
    {
      std::cout << "using trackball X19"
                << "\n";
      pDevice = "/dev/input/mouse0";
      std::cout << "using mouse0"
                << "\n";
      printf(" Opening  %s\n", pDevice);
    }
    else if (!trackball_name.compare("X13"))
    {
      std::cout << "using trackball X13"
                << "\n";
      pDevice = "/dev/input/mouse1";
      std::cout << "using mouse1"
                << "\n";
      printf(" Opening  %s\n", pDevice);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "wrong name of trackball. Using all mouse0 as input.");
    }

    //  const char *pDevice = "/dev/input/by-id/usb-Logitech_USB-PS_2_Optical_Mouse-mouse";
    // const char *pDevice = "/dev/input/by-id/usb-Cursor_Controls_Ltd_Cursor_Controls_Trackball-mouse";

    // Open Mouse
    fd = open(pDevice, O_RDONLY);
    if (fd == -1)
    {
      printf("ERROR Opening  %s\n", pDevice);
      close(fd);
    }

    int left, middle, right;
    signed char x, y;
    rclcpp::Time start_time = this->get_clock()->now();
    rclcpp::Time publish_time = this->get_clock()->now();
    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Time last_time = this->get_clock()->now();

    rclcpp::Time time1 = this->get_clock()->now();
    rclcpp::Time time2 = this->get_clock()->now();

    while (1)
    {
      counter++;
      bytes = read(fd, data, sizeof(data));
      current_time = this->get_clock()->now();
      if (bytes > 0)
      {
        // printf("data: %02x\n",data);
        left = data[0] & 0x1;
        right = data[0] & 0x2;
        middle = data[0] & 0x4;

        x = data[1];
        y = data[2];
        sum_x += x;
        sum_y += y;
        deltaT = (this->get_clock()->now() - publish_time).nanoseconds() / 1000000;

        last_time = current_time;
        // printf("counter= %i, readTime [µs]= %f, deltaT [ms]= %f, sum_x=%d, sum_y=%d\n", counter, deltaT_temp, deltaT, sum_x, sum_y);
        //printf("sum_x=%d, sum_y=%d\n", sum_x, sum_y);
        counter = 0;
        if (deltaT >= 20)
        {
          publish_time = this->get_clock()->now();
        }
        auto message = geometry_msgs::msg::PointStamped();
        message.point.x = (double)x;
        message.point.y = (double)y;
        message.point.z = deltaT;
        message.header.stamp = this->get_clock()->now();
        tick_publisher_->publish(message);
      }
    }
  }
  TrackballInterface()
      : Node("trackball_interface")
  {
    printf("constructor...");

    this->declare_parameter("trackball_name", "X19");

    trackball_name = this->get_parameter("trackball_name").as_string();
    std::string topic_name_prefix = "/trackball";
    // const str::string topic_name_suffix1 = "/position";
    std::string topic_name_position = topic_name_prefix;
    std::string topic_name_ticks = topic_name_prefix;
    // don't add trackball_name if it is "X19" ==> default topic
    if (trackball_name.compare("X19"))
    {
      topic_name_position.append(trackball_name);
      topic_name_ticks.append(trackball_name);
    }
    topic_name_position.append("/position");
    topic_name_ticks.append("/ticks");

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(topic_name_position, 10);
    tick_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(topic_name_ticks, 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&TrackballInterface::timer_callback, this), timer_cb_group_);

    zeroService =
        this->create_service<std_srvs::srv::Trigger>("trackball_interface/setZero", std::bind(&TrackballInterface::setToZeroCB, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
                                                     service_cb_group_);
    printf("starting...");
    std::thread read_mouse_thread(&TrackballInterface::read_mouse, this);
    read_mouse_thread.detach();
  }

  ~TrackballInterface()
  {
    printf("destructor");
    close(fd);
  }

private:
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  void timer_callback()
  {
    auto message = geometry_msgs::msg::PointStamped();
    message.point.x = (double)sum_x;
    message.point.y = (double)sum_y;
    message.point.z = 0;
    message.header.stamp = this->get_clock()->now();
    odom_publisher_->publish(message);
  }
  std::string trackball_name;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroService;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tick_publisher_;

  void setToZeroCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::cout << "setZero service" << std::endl;
    this->sum_x = 0;
    this->sum_y = 0;
    response->success = true;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TrackballInterface>();
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  return 0;
}
