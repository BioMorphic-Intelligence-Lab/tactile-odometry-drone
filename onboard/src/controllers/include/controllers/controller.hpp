#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/timesync_status.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node
{
public:
    Controller();

    /**
     * @brief Publish a trajectory setpoint
     */
    virtual void publish_trajectory_setpoint() = 0;
    
    /**
     * @brief Send a command to land
     */
    void land();

    /**
     * @brief Normalize angle to -180 < angle <= 180
     */
    double normalize_angle(double angle);

    uint64_t get_timestamp();

    rclcpp::Time _beginning;
    bool _taken_off = false, _landed = false;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_publisher;

private: 

    uint8_t _nav_state, _arming_state;
   
    uint _offboard_setpoint_counter;

    double _frequency;

    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_subscription;
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

    // Time offset
    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;

    /* Callback Functions */
    void _timer_callback();


    /**
     * @brief Publish the offboard control mode.
     *        For this example, only position and altitude controls are active.
     */
    void _publish_offboard_control_mode();

    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Send a command to takeoff
     */
    void takeoff();
    
    void _publish_vehicle_command(uint16_t command,
                                  float param1 = 0.0,
                                  float param2 = 0.0,
                                  float param3 = 0.0,
                                  float param4 = 0.0,
                                  float param5 = 0.0,
                                  float param6 = 0.0,
                                  float param7 = 0.0);

};


