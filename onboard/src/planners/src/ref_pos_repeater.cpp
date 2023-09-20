#include <Eigen/Dense>

#include "ref_pos_repeater.hpp"
#include "common/common.hpp"

using namespace personal;

RefPosRepeater::RefPosRepeater()
    : Node("RefPosRepeater"), _offboard_setpoint_counter(0)
{   

    /* Declare all the parameters */
    this->declare_parameter("frequency", 10.0);
    this->declare_parameter("sub_topic", "/ref_pose");

    /* Actually get all the parameters */
    this->_frequency =  this->get_parameter("frequency").as_double();

    /* Init Timer and subscribers*/
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                            std::bind(&RefPosRepeater::_timer_callback, this));
    this->_ref_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("sub_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&RefPosRepeater::_ref_callback, this, std::placeholders::_1));

    this->_timesync_subscription = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        "/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&RefPosRepeater::_timesync_callback, this, std::placeholders::_1));
    
    /* Init Publishers */
    this->_offboard_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    this->_trajectory_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    this->_vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    /* Remember the beginning time stamp */
    this->_beginning = this->now();

}


/* Callback Functions */
void RefPosRepeater::_timer_callback()
{
    if (_offboard_setpoint_counter == 20) 
    {
        /* On the real system we want to arm and change mode using the remote control
            Uncomment this for the SITL e.g. automatic arming and switch to offboard mode */
            
        // Change to Offboard mode after 10 setpoints
        //this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        //this->arm();
        //this->takeoff();

    }

    // offboard_control_mode
    this->_publish_offboard_control_mode();

    // stop the counter after reaching 11
    if (_offboard_setpoint_counter < 21) {
        _offboard_setpoint_counter++;
    }

}

/**
 *  @brief  Callback function for ref position in ros frame
 * Transforms it into px4 frame and publishes it on the px4 topic */
void RefPosRepeater::_ref_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    px4_msgs::msg::TrajectorySetpoint px4_msg{};

    /* In the beginning we just want to take off */
    if((this->_beginning - this->now()).seconds() < 20.0)
    {
        px4_msg.position = {0.0, 0.0, -1.7};
        px4_msg.yaw = 0.0;
    }
    /* Then we forward the transformed reference position */
    else
    {   
        // Transform the position
        Eigen::Vector3d enu_position = {msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z};
        Eigen::Vector3d ned_position = common::enu_2_ned(enu_position);

        // Transform the orientation    
        Eigen::Quaterniond enu_q = {msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z};
        Eigen::Quaternion ned_q = common::enu_2_ned(enu_q);

        // Save to px4_msg
        px4_msg.position = {(float)ned_position.x(),
                            (float)ned_position.y(),
                            (float)ned_position.z()};
        // This assumes the quaternion only describes yaw.
        px4_msg.yaw = 2 * acos(ned_q.w());
    }
 


    // Get the current timestamp
    px4_msg.timestamp = this->get_timestamp();

    // Publish the reference position
    this->_trajectory_publisher->publish(px4_msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void RefPosRepeater::_publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_timestamp();
    _offboard_publisher->publish(msg);
}


void RefPosRepeater::_timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
    this->_timestamp_local = std::chrono::steady_clock::now();
    this->_timestamp_remote.store(msg->timestamp);
}

uint64_t RefPosRepeater::get_timestamp()
{
    auto now = std::chrono::steady_clock::now();
    return this->_timestamp_remote.load() + std::chrono::round<std::chrono::microseconds>(now - this->_timestamp_local).count();
}

/**
 * @brief Send a command to Arm the vehicle
 */
void RefPosRepeater::arm()
{
    this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/**
 * @brief Send a command to Disarm the vehicle
 */
void RefPosRepeater::disarm()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}
/**
 * @brief Send a command to takeoff
 */
void RefPosRepeater::takeoff()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param7 = 2.5;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    _vehicle_command_pub->publish(msg);

    this->_taken_off = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}
/**
 * @brief Send a command to land
 */
void RefPosRepeater::land()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(this->get_logger(), "Land command send");
}


void RefPosRepeater::_publish_vehicle_command(uint16_t command,
                                                            float param1,
                                                            float param2,
                                                            float param3,
                                                            float param4,
                                                            float param5,
                                                            float param6,
                                                            float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    this->_vehicle_command_pub->publish(msg);
}
