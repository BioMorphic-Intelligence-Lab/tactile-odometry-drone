#include "planner.hpp"

#include "rclcpp/time.hpp"
#include "kinematics/kinematics.hpp"

using namespace personal;

Planner::Planner()
    : Node("Planner"), JS_THRESHOLD(0.001),
      output_q(0.0, 0.0, 0.0, 1.0) // MS: make the threshold a paramteter
{

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("desired_linear_joint_pos", -0.01); // position in m
    this->declare_parameter("v_approach", 0.25);                // Approach velocity
    this->declare_parameter("alignment_threshold", M_PI / 180.0 * 15);
    this->declare_parameter("yaw_rate", M_PI / 180.0 * 3); // 10 Degree/s
    this->declare_parameter("align", true);
    this->declare_parameter("start_point", std::vector<double>({0.52, 2.46, 1.68} /*{-0.31, 1.95, 1.85}*/));
    this->declare_parameter("joint_topic", "/joint_state");
    this->declare_parameter("pose_topic", "/mocap_pose");
    this->declare_parameter("ee_topic", "/ee_pose");
    this->declare_parameter("trackball_topic", "/trackballX19/position");
    this->declare_parameter("pub_topic", "/ref_pose");

    /* Actually get all the parameters */
    this->_yaw_rate = this->get_parameter("yaw_rate").as_double();
    this->_frequency = this->get_parameter("frequency").as_double();
    this->_alignment_threshold = this->get_parameter("alignment_threshold").as_double();
    this->_v_approach = this->get_parameter("v_approach").as_double();
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
    this->_trackball_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
        this->get_parameter("trackball_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Planner::_trackball_callback, this, std::placeholders::_1));

    /* Init Publishers */
    this->_setpoint_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("pub_topic").as_string(), 10);
    this->_setpoint_publisher_ee = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ref_pose/ee", 10);

    /* Init TF publisher */
    this->_tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /* Init current joint state variable to avoid segmentation fault */
    this->_curr_js.position = {0.0, 0.0};
    this->_curr_js.velocity = {0.0, 0.0};
    this->_last_js = this->_curr_js;

    /* Init the timestamp to some time in the future value until we establish contact */
    this->_beginning = this->now() + rclcpp::Duration(10000, 0);
    this->_approach_beginning = this->now();

    /* Services */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trackball_set_zero =
        this->create_client<std_srvs::srv::Trigger>("trackball_interface/setZero");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    while (!trackball_set_zero->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    trackball_set_zero->async_send_request(request);
}

void Planner::_get_uav_to_ee_position()
{
    Eigen::Vector3d mocap_pos = this->_current_position;
    Eigen::Vector3d ee_pos= this->_current_ee_position;

    if (this->_ee_offsets.size() <= 50)
    {
        this->_ee_offsets.push_back((ee_pos - mocap_pos));
    }
    if (this->_ee_offsets.size() == 50)
    {
        this->_ee_offset = this->_current_quat.toRotationMatrix() * std::reduce(this->_ee_offsets.begin(), this->_ee_offsets.end()) / this->_ee_offsets.size();
    }
}

// needs to be applied on unaligned position and need to be aligned afterwards
double Planner::_control_contact_force(float linear_joint, float desired_joint)
{
    float p_gain = 1.0; // should be less than 1, as joint values are in m
    float d_gain = 0.1;
    float joint_velocity = 0.5 * (this->_curr_js.velocity[0] + this->_last_js.velocity[0]);

    this->_last_js = this->_curr_js;

    return p_gain * (linear_joint - desired_joint) - d_gain * joint_velocity;
}

/**
 * @brief Align UAV to be perpendicular to wall (i.e. encoderYaw == 0). The function is designed to run permanentely after the trajectory position has been set
 * Inputs:
 *      pos_WE: position of End-Effector-Tip in Wall-Frame from trajectory_setpoint()
        encoderYaw: reading of encoder (in rad)
        quat_mocap_q: current mocap orientation
        pos_BE:  position offset from UAV to End-Effector
* Outputs:
*   pos_IB: desired position of UAV-Body in world-frame
*   quat_IB: new desired orientation of UAV
 */
void Planner::_align_to_wall(Eigen::Quaterniond &quat_IB, Eigen::Vector3d &pos_IB, Eigen::Vector3d pos_WE, Eigen::Vector3d pos_BE, float encoder_yaw, Eigen::Quaterniond quat_mocap_q)
{
    // Currently unused
    (void)quat_mocap_q;

    double yaw = common::yaw_from_quaternion_y_align(quat_IB);
    double dt = 1.0 / this->_frequency;
    double increment = -copysign(this->_yaw_rate, encoder_yaw) * dt;

    if (fabs(5.0 * M_PI / 180.0) > fabs(encoder_yaw))
    {
        increment = 0.0;
    }

    // get rotation about z-axis that preserves x-y-heading of y-axis
    // const Eigen::Matrix3d R_IB = quat_IB.toRotationMatrix();
    // double yaw = -atan2(R_IB(1, 2), R_IB(2, 2));
    float yaw2 = yaw + increment;
    auto &clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         clk,
                         500,
                         "yaw1 %f, yaw2 %f, increment %f, encoder yaw %f", yaw, yaw2, increment, encoder_yaw);
    // Rotation Matrix between World/Inertial (I) and Wall (W)
    // Eigen::Matrix3d R_IW = quat_IB.toRotationMatrix() * common::quaternion_from_euler(0.0, 0.0, -encoder_yaw);
    const Eigen::Matrix3d R_IB_z = common::rot_z(yaw2);
    const Eigen::Matrix3d R_BW_z = common::rot_z(-encoder_yaw);
    const Eigen::Matrix3d R_IW_z = R_IB_z * R_BW_z;

    // return valies quat_IB and pos_IB
    // quat_IB = common::quaternion_from_euler(0.0, 0.0, yaw2);
    // return position and orientation of uav
    // pos_IB = R_IW_z * pos_WE - R_IB_z * pos_BE;

    kinematics::inverse_kinematics(R_IW_z, Eigen::Vector3d(0, 0, 0), pos_WE, this->_curr_js.position, 0.0, 0.0, 0.0,
                                   R_IB_z, pos_IB);
    quat_IB = Eigen::Quaterniond(R_IB_z);
}

/* Callback Functions */
void Planner::_timer_callback()
{
    /* Init the Message and time stamp it */
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();

    /* Extract current state */
    Eigen::Vector3d curr_position = this->_current_position;
    Eigen::Quaterniond curr_quat = this->_current_quat;
    std::vector<double> joint_pos = this->_curr_js.position;

    /* Put in the position of the planner */
    Eigen::Vector3d position = this->get_trajectory_setpoint();

    /*publish original trajectory pose*/
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();
    this->_setpoint_publisher_ee->publish(msg);

    if (this->_in_contact)
    {
        /* check if UAV is aligned to all*/
        this->_is_aligned = (this->_is_aligned) ||
                            fabs(fmod(joint_pos[1], 2 * M_PI)) < this->_alignment_threshold;
    }
    else
    {
        // reset position offset if contact is lost
        this->_position_offset = 0.0;

        /* check if UAV is in contact*/
        this->_in_contact = this->_detect_contact();
    }

    if (this->_is_aligned && this->_in_contact)
    {
        // if alignment is lost, _position_offset will remain at last value to pprevent jumps. it will be reset, if conact is lost
        this->_position_offset = _control_contact_force(joint_pos[0],
                                                        this->_desired_linear_joint_pos);
    }
    else
    {
        // set beginning 1s to future, so it starts 1s after contact is established and alignment is done
        this->_beginning = this->now() + rclcpp::Duration(1, 0);

        /* get ee_offset*/
        this->_get_uav_to_ee_position();
    }

    // allways add position offset (it will be 0 if not wanted)
    position.y() += this->_position_offset;

    /* Check if we already are in contact and if we want to align with the wall */
    if (this->_align && this->_in_contact)
    {

        Eigen::Vector3d aligned_position;

        /* Find the aligned position and orientation */
        _align_to_wall(output_q, aligned_position, position,
                       this->_ee_offset + this->_curr_js.position[0] * Eigen::Vector3d::UnitY(),
                       this->_curr_js.position[1],
                       curr_quat);

        /* Transform to start point */
        RCLCPP_DEBUG(this->get_logger(), "%f %f %f", aligned_position.x(), aligned_position.y(), aligned_position.z());
        aligned_position += this->_start_point;

        /* Add it to the message */
        // Eigen::Quaterniond output_q = common::quaternion_from_euler(0, 0, output_yaw);
        msg.pose.orientation.w = output_q.w();
        msg.pose.orientation.x = output_q.x();
        msg.pose.orientation.y = output_q.y();
        msg.pose.orientation.z = output_q.z();

        msg.pose.position.x = aligned_position.x();
        msg.pose.position.y = aligned_position.y();
        msg.pose.position.z = aligned_position.z();
    }
    /* Otherwise the trajectory has not started yet and
     * we fly towards the start position. For this we command reference positions that are
     * in the direction of the start position at a distance of v_approach * dt */
    else
    {

        double t = (this->now() - this->_approach_beginning).seconds() - 10.0;

        if (t > 0)
        {
            position.z() += this->_start_point.z();
            Eigen::Vector3d pos = this->_v_approach * t * Eigen::Vector3d(this->_start_point.x(), this->_start_point.y(), 0.0).normalized();
            if (pos.y() > this->_start_point.y())
                pos = this->_start_point;
            position += pos;
        }

        msg.pose.position.x = position.x();
        msg.pose.position.y = position.y();
        msg.pose.position.z = position.z();
        msg.pose.orientation.w = 0;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 1.0;
    }

    /* Finally publish the message */
    this->_setpoint_publisher->publish(msg);

    /* Publish TF */
    double joint_state[2] = {this->_curr_js.position[0], this->_curr_js.position[1]};
    this->_publish_tf(curr_quat.toRotationMatrix(), curr_position,
                      joint_state);
}

void Planner::_publish_tf(Eigen::Matrix3d R_IB, 
                          Eigen::Vector3d p_IB,
                          double joint_state[2])
{
    Eigen::Matrix3d R_IE, R_IT, R_IO, R_IW;
    Eigen::Vector3d p_IE, p_IT, p_IO; 

    kinematics::forward_kinematics(R_IB, p_IB, joint_state, 0.0, 0.0,
                        R_IE, R_IT, R_IO, R_IW, p_IE, p_IT, p_IO);


    geometry_msgs::msg::TransformStamped IE, IT, IO, IW;
    
    // Set Timestamps
    auto ts = this->now();
    IE.header.stamp = ts;
    IT.header.stamp = ts;
    IO.header.stamp = ts;
    IW.header.stamp = ts;

    // Set Frame IDs
    IE.header.frame_id = "world";
    IT.header.frame_id = "world";
    IO.header.frame_id = "world";
    IW.header.frame_id = "world";

    // Set Child Frame ID
    IE.child_frame_id = "EE";
    IT.child_frame_id = "TCP";
    IO.child_frame_id = "Odom";
    IW.child_frame_id = "Wall";

    // Set Transform
    IE.transform = this->_transform_from_eigen(Eigen::Quaterniond(R_IE), p_IE);
    IT.transform = this->_transform_from_eigen(Eigen::Quaterniond(R_IT), p_IT);
    IO.transform = this->_transform_from_eigen(Eigen::Quaterniond(R_IO), p_IO);
    IW.transform = this->_transform_from_eigen(Eigen::Quaterniond(R_IW), p_IO);

    this->_tf_broadcaster->sendTransform(IE);
    this->_tf_broadcaster->sendTransform(IT);
    this->_tf_broadcaster->sendTransform(IO);
    this->_tf_broadcaster->sendTransform(IW);
}

geometry_msgs::msg::Transform Planner::_transform_from_eigen(Eigen::Quaterniond rot, Eigen::Vector3d pos)
{
    geometry_msgs::msg::Transform T;
    T.translation.x = pos.x();
    T.translation.y = pos.y();
    T.translation.z = pos.z();
    T.rotation.w = rot.w();
    T.rotation.x = rot.x();
    T.rotation.y = rot.y();
    T.rotation.z = rot.z();

    return T;
}

void Planner::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->_curr_js = *msg;
    this->_curr_js.position[1] = common::normalize_angle(this->_curr_js.position[1]);
}

void Planner::_mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    auto pose = *msg;
    this->_current_position = Eigen::Vector3d(pose.pose.position.x,
                                           pose.pose.position.y,
                                           pose.pose.position.z);
    
    this->_current_quat = Eigen::Quaterniond(pose.pose.orientation.w,
                                            pose.pose.orientation.x,
                                            pose.pose.orientation.y,
                                            pose.pose.orientation.z);
}
void Planner::_ee_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    auto pose = *msg;
    this->_current_ee_position = Eigen::Vector3d(pose.pose.position.x,
                                              pose.pose.position.y,
                                              pose.pose.position.z);
    
    this->_current_ee_quat = Eigen::Quaterniond(pose.pose.orientation.w,
                                               pose.pose.orientation.x,
                                               pose.pose.orientation.y,
                                               pose.pose.orientation.z);
}

void Planner::_trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    this->_trackball_pos.x() = msg->point.x;
    this->_trackball_pos.y() = msg->point.y;
    this->_trackball_pos.z() = msg->point.z;
}

bool Planner::_detect_contact()
{
    const bool force_over_threshold = fabs(this->_curr_js.position[0]) > JS_THRESHOLD;
    const bool trackball_over_threshold = this->_trackball_pos.norm() > 0.01;

    const bool all_over_threshold = force_over_threshold || trackball_over_threshold;

    if (all_over_threshold && !this->_contact_temp) // rising edge
    {
        this->_time_of_first_contact = this->now();
    }
    if (!all_over_threshold)
    {
        this->_time_of_first_contact = this->now() + rclcpp::Duration(5, 0);
    }
    this->_contact_temp = all_over_threshold;

    return ((this->now() - this->time_of_first_contact).seconds() > this->_minimum_contact_duration);
}