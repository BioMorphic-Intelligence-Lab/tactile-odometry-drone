#include "planner.hpp"

#include "rclcpp/time.hpp"

Planner::Planner()
    : Node("Planner"), JS_THRESHOLD(0.01) // MS: make the threshold a paramteter
{

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("desired_linear_joint_pos", 0.03); // position in m
    this->declare_parameter("alignment_threshold", M_PI / 180.0 * 15);
    this->declare_parameter("yaw_rate", M_PI / 180.0 * 10); // 10 Degree/s
    this->declare_parameter("align", false);
    this->declare_parameter("start_point", std::vector<double>({0, 1.0, 1.8}));
    this->declare_parameter("joint_topic", "/joint_pose");
    this->declare_parameter("pose_topic", "/mocap_pose");
    this->declare_parameter("ee_topic", "/ee_pose");
    this->declare_parameter("pub_topic", "/ref_pose");

    /* Actually get all the parameters */
    this->_yaw_rate = this->get_parameter("yaw_rate").as_double();
    this->_frequency = this->get_parameter("frequency").as_double();
    this->_alignment_threshold = this->get_parameter("alignment_threshold").as_double();
    this->_desired_linear_joint_pos = this->get_parameter("desired_linear_joint_pos").as_double();
    this->_align = this->get_parameter("align").as_bool();
    this->_ee_offset << 0, 0.2, -0.1;
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
    this->_ee_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("ee_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_ee_callback, this, std::placeholders::_1));

    /* Init Publishers */
    this->_setpoint_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pub_topic").as_string(), 10);

    /* Init current joint state variable to avoid segmentation fault */
    this->_curr_js.position = {0.0, 0.0};

    /* Init the timestamp to some time in the future value until we establish contact */
    this->_beginning = this->now() + rclcpp::Duration(10000, 0);
}

void Planner::get_uav_to_ee_position()
{
    Eigen::Vector3d mocap_pos(this->_curr_pos.pose.position.x, this->_curr_pos.pose.position.y, this->_curr_pos.pose.position.z);
    Eigen::Vector3d ee_pos(this->_curr_ee_pos.pose.position.x, this->_curr_ee_pos.pose.position.y, this->_curr_ee_pos.pose.position.z);
    Eigen::Vector3d mean;

    if (this->_ee_offsets.size() <= 50)
    {
        this->_ee_offsets.push_back(ee_pos - mocap_pos);
    }
    if (this->_ee_offsets.size() == 50)
    {
        this->_ee_offset = std::reduce(this->_ee_offsets.begin(), this->_ee_offsets.end()) / this->_ee_offsets.size();
    }
}

// needs to be applied on unaligned position and need to be aligned afterwards
double Planner::control_contact_force(float linear_joint, float desired_joint)
{
    float p_gain = 0.5; // should be less than 1, as joint values are in m
    return p_gain * (linear_joint - desired_joint);
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
    if (encoder_yaw != 0)
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
    pos_IB = pos_IE - common::rot_z(yaw) * (pos_BE);
    yaw_IB = yaw;
}

/* Callback Functions */
void Planner::_timer_callback()
{
    /* Init the Message and time stamp it */
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();

    /* Extract current state */
    geometry_msgs::msg::PoseStamped curr_pos = this->_curr_pos;
    std::vector<double> joint_pos = this->_curr_js.position;

    /* Put in the position of the planner */
    Eigen::Vector3d position = this->get_trajectory_setpoint()
                             + this->_start_point; // Transform to start point

    if (this->_in_contact)
    {
        /* check if UAV is aligned to all*/
        this->_is_aligned = fabs(fmod(joint_pos[1], 2 * M_PI)) < this->_alignment_threshold;
    }
    else
    {
        // reset position offset if contact is lost
        this->_position_offset = 0.0;

        /* check if UAV is in contact*/
        this->_in_contact = fabs(this->_curr_js.position[0]) > JS_THRESHOLD;
    }

    if (this->_is_aligned && this->_in_contact)
    {
        // if alignment is lost, _position_offset will remain at last value to pprevent jumps. it will be reset, if conact is lost
        this->_position_offset = control_contact_force(joint_pos[0], this->_desired_linear_joint_pos);
    }
    else
    {
        // set beginning 1s to future, so it starts 1s after contact is established and alignment is done
        this->_beginning = this->now() + rclcpp::Duration(1, 0);

        /* get ee_offset*/
        this->get_uav_to_ee_position();
    }

    // allways add position offset (it will be 0 if not wanted)
    position.y() += this->_position_offset;

    /* Check if we already are in contact and if we want to align with the wall */
    if (this->_align && this->_in_contact)
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
        align_to_wall(output_yaw, aligned_position, position,
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
    /* Otherwise we just forward the position */
    else
    {
        msg.pose.position.x = position.x();
        msg.pose.position.y = position.y();
        msg.pose.position.z = position.z();
    }

    /* Finally publish the message */
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
void Planner::_ee_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->_curr_ee_pos = *msg;
}