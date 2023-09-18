#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"

using namespace std::chrono_literals;

class MocapForwarder : public rclcpp::Node
{
public:
    MocapForwarder();

private:
    /**
     * @brief Publish a trajectory setpoint
     */
    void _handle_visual_odometry(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    std::string _sub_topic;

private: 

    /* Callback Functions */
    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);    uint64_t 
    
    get_timestamp();

    /* Publishers and Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _vehicle_odom_sub;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_subscription;
    
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odom_pub;

    // Time offset
    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;


};