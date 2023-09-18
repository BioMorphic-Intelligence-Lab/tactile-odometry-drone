#include "planner.hpp"

#include "rclcpp/time.hpp"

Planner::Planner()
    : Node("Planner"), JS_THRESHOLD(0.01)
{   

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("yaw_rate", M_PI / 180.0 * 10); // 10 Degree/s
    this->declare_parameter("align", true);
    this->declare_parameter("ee_offset", std::vector<double>({0, 0.2, -0.1}));
    this->declare_parameter("start_point", std::vector<double>({0, 1.0, 1.8}));
    this->declare_parameter("joint_topic", "/JointState");
    this->declare_parameter("pose_topic", "/MocapPose");
    this->declare_parameter("pub_topic", "/ref_pose");

    /* Actually get all the parameters */
    this->_yaw_rate = this->get_parameter("yaw_rate").as_double();
    this->_frequency =  this->get_parameter("frequency").as_double();
    this->_align = this->get_parameter("align").as_bool();
    std::vector<double> ee_offset = this->get_parameter("ee_offset").as_double_array();
    this->_ee_offset << ee_offset.at(0), ee_offset.at(1), ee_offset.at(2);
    std::vector<double> start_point = this->get_parameter("start_point").as_double_array();
    this->_start_point << start_point.at(0), start_point.at(1), start_point.at(2);
    
 
    /* Init Timer and subscribers*/
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                            std::bind(&Planner::_timer_callback, this));
    this->_joint_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        this->get_parameter("joint_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_joint_callback, this, std::placeholders::_1));
    this->_mocap_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pose_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_mocap_callback, this, std::placeholders::_1));
    
    /* Init Publishers */
    this->_setpoint_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pub_topic").as_string(), 10);

    /* Init current joint state variable to avoid segmentation fault */
    this->_curr_js.position = {1000, 0.0};

    /* Init the timestamp to some time in the future value until we establish contact */
    this->_beginning = this->now() + rclcpp::Duration(10000, 0);



}

/**
 * @brief Align UAV to be perpendicular to wall (i.e. encoderYaw == 0). The function is designed to run permanentely after the trajectory position has been set
 * Inputs:
 *      pos_IE: position of End-Effector-Tip in World-Frame
        encoderYaw: reading of encoder (in rad)
        mocapYaw: current yaw angle of UAV
        pos_BE:  position offset from UAV to End-Effector
* Outputs:
*   pos_IB: updated position of UAV
*   yaw_IB: updated yaw of UAV
 */
void Planner::align_to_wall(float &yaw_IB, Eigen::Vector3d &pos_IB, Eigen::Vector3d pos_IE, Eigen::Vector3d pos_BE, float encoder_yaw, float mocap_yaw)
{
    float yaw = 0;
    // If the encoder yaw is zero we do not want to align and only transform EE ref pose to Base ref pose
    if(encoder_yaw != 0)
    {    
        float increment = copysign(this->_yaw_rate / this->_frequency, encoder_yaw);

        // limit increment to prevent overshoot
        if (abs(increment) > abs(encoder_yaw))
        {
            increment = encoder_yaw;
        }

        yaw = mocap_yaw + increment;
    }

    // return position and orientation of uav
    pos_IB = (pos_IE + this->_start_point) - common::rot_z(yaw) * (pos_BE);
    yaw_IB = yaw;
}


/* Callback Functions */
void Planner::_timer_callback()
{
    /* Init the Message and time stamp it */
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "World";
    msg.header.stamp = this->now();

    /* Extract current state */
    geometry_msgs::msg::PoseStamped curr_pos = this->_curr_pos;
    std::vector<double> joint_pos = this->_curr_js.position;

    /* Put in the position of the planner */
    std::vector<double> position = this->get_trajectory_setpoint();
    
    /* Check if we already are in contact and if we want to align with the wall */
    if(this->_align && this->_in_contact)
    {
        /* Init input and ouput variables */
        float output_yaw = 0;
        float curr_yaw = common::yaw_from_quaternion(
                Eigen::Quaterniond(curr_pos.pose.orientation.w,
                                curr_pos.pose.orientation.x,
                                curr_pos.pose.orientation.y,
                                curr_pos.pose.orientation.z));
        Eigen::Vector3d aligned_position;
        
        /* Find the aligned position and orientation */
        align_to_wall(output_yaw, aligned_position,
            Eigen::Vector3d(position.at(0),
                            position.at(1),
                            position.at(2)),
            this->_ee_offset + this->_curr_js.position[0] * Eigen::Vector3d::UnitY(), 
            this->_curr_js.position[1],
            curr_yaw);

        /* Add it to the message */
        Eigen::Quaterniond output_q = common::quaternion_from_euler(0, 0, output_yaw);
        msg.pose.orientation.w = output_q.w();
        msg.pose.orientation.x = output_q.x();
        msg.pose.orientation.y = output_q.y();
        msg.pose.orientation.z = output_q.z();

        msg.pose.position.x = aligned_position.x();
        msg.pose.position.y = aligned_position.y();
        msg.pose.position.z = aligned_position.z();
    }
    /* Otherwise we leave just use the start pose and only check if we're already in contact */
    else
    {
        if(this->_curr_js.position[0] < JS_THRESHOLD)
        {
            this->_in_contact = true;
            this->_beginning = this->now();
        }

        msg.pose.position.x = this->_start_point.x();
        msg.pose.position.y = this->_start_point.y();
        msg.pose.position.z = this->_start_point.z();
    }
    this->_setpoint_publisher->publish(msg);
}


void Planner::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->_curr_js = *msg;
}

void Planner::_mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->_curr_pos = *msg;
}