#include "mocap_forwarder.hpp"
#include "common/common.hpp"

#include <Eigen/Dense>

MocapForwarder::MocapForwarder():rclcpp::Node("mocap_forwarder")
{
    /* Declare all the parameters */
    this->declare_parameter("sub_topic", "/mocap_pose");

    this->_sub_topic = this->get_parameter("sub_topic").as_string();

    /* Init Subscriber*/
    this->_vehicle_odom_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                this->_sub_topic, rclcpp::SensorDataQoS(), std::bind(
                &MocapForwarder::_handle_visual_odometry, this, std::placeholders::_1));
    this->_timesync_subscription = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        "/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&MocapForwarder::_timesync_callback, this, std::placeholders::_1));

    /* Init Publisher */
    this->_vehicle_odom_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", 10);
}

void MocapForwarder::_handle_visual_odometry(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Eigen::Vector3d enu_p(msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z);
    Eigen::Vector3d ned_p = common::enu_2_ned(enu_p);

    Eigen::Quaterniond enu_q(msg->pose.orientation.w,
                             msg->pose.orientation.x,
                             msg->pose.orientation.y,
                             msg->pose.orientation.z);
    Eigen::Quaterniond ned_q = common::enu_2_ned(enu_q);

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = this->get_timestamp();
    odom.timestamp_sample = odom.timestamp;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    odom.position = {(float)ned_p.x(), (float)ned_p.y(), (float)ned_p.z()};
    odom.q = {(float)ned_q.x(), (float)ned_q.y(), (float)ned_q.z(), (float)ned_q.w()};

    this->_vehicle_odom_pub->publish(odom);
}

void MocapForwarder::_timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
    this->_timestamp_local = std::chrono::steady_clock::now();
    this->_timestamp_remote.store(msg->timestamp);
} 

uint64_t MocapForwarder::get_timestamp()
{
    auto now = std::chrono::steady_clock::now();
    return this->_timestamp_remote.load() 
        + std::chrono::round<std::chrono::microseconds>(now 
            - this->_timestamp_local).count();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<MocapForwarder>());
  rclcpp::shutdown();
  return 0;
}
