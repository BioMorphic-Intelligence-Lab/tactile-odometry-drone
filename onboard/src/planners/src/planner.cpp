#include "planner.hpp"

#include "rclcpp/time.hpp"

using namespace personal;

Planner::Planner()
    : Node("Planner"), JS_THRESHOLD(0.001) // MS: make the threshold a paramteter
{

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("desired_linear_joint_pos", -0.01); // position in m
    this->declare_parameter("v_approach", 0.1);                 // Approach velocity
    this->declare_parameter("alignment_threshold", M_PI / 180.0 * 15);
    this->declare_parameter("yaw_rate", M_PI / 180.0 * 10); // 10 Degree/s
    this->declare_parameter("align", true);
    this->declare_parameter("start_point", std::vector<double>({-1.05, 2.375, 1.87}));
    this->declare_parameter("joint_topic", "/joint_state");
    this->declare_parameter("pose_topic", "/mocap_pose");
    this->declare_parameter("ee_topic", "/ee_pose");
    this->declare_parameter("trackball_topic", "/trackball/position");
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

    /* Init current joint state variable to avoid segmentation fault */
    this->_curr_js.position = {0.0, 0.0};

    /* Init the timestamp to some time in the future value until we establish contact */
    this->_beginning = this->now() + rclcpp::Duration(10000, 0);

    /* Services*/
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
    Eigen::Vector3d mocap_pos(this->_curr_pose.pose.position.x, this->_curr_pose.pose.position.y, this->_curr_pose.pose.position.z);
    Eigen::Vector3d ee_pos(this->_curr_ee_pose.pose.position.x, this->_curr_ee_pose.pose.position.y, this->_curr_ee_pose.pose.position.z);

    if (this->_ee_offsets.size() <= 50)
    {
        this->_ee_offsets.push_back((ee_pos - mocap_pos));
    }
    if (this->_ee_offsets.size() == 50)
    {
        const auto pose = this->_curr_pose.pose.orientation;
        const Eigen::Quaterniond q(pose.w, pose.x, pose.y, pose.z);
        this->_ee_offset = q.toRotationMatrix() * std::reduce(this->_ee_offsets.begin(), this->_ee_offsets.end()) / this->_ee_offsets.size();
    }
}

// needs to be applied on unaligned position and need to be aligned afterwards
double Planner::_control_contact_force(float linear_joint, float desired_joint)
{
    float p_gain = 1; // should be less than 1, as joint values are in m
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
void Planner::_align_to_wall(Eigen::Quaterniond &quat_IB, Eigen::Vector3d &pos_IB, Eigen::Vector3d pos_WE, Eigen::Vector3d pos_BE, float encoder_yaw, Eigen::Quaterniond quat_mocap_q)
{

    Eigen::Matrix4d W;
    Eigen::Vector4d quat_mocap(quat_mocap_q.w(), quat_mocap_q.x(), quat_mocap_q.y(), quat_mocap_q.z());
    double increment = this->_yaw_rate;
    increment = -copysign(increment, encoder_yaw);
    double dt = 1.0 / this->_frequency;
    if (abs(increment * dt) > abs(encoder_yaw))
    {
        increment = encoder_yaw / dt;
    }
    W << 1, 0, 0, -increment,
        0, 1, increment, 0,
        0, -increment, 1, 0,
        increment, 0, 0, 1;

    Eigen::Vector4d quat_IB_4d = (Eigen::Matrix4d::Identity() + 0.5 * W * dt) * quat_mocap;

    quat_IB = Eigen::Quaterniond(quat_IB_4d[0], quat_IB_4d[1], quat_IB_4d[2], quat_IB_4d[3]).normalized();

    // Rotation Matrix between World/Inertial (I) and Wall (W)
    Eigen::Matrix3d R_IW = quat_IB.toRotationMatrix() * common::quaternion_from_euler(0.0, 0.0, -encoder_yaw);
    // return position and orientation of uav
    pos_IB = R_IW * (pos_WE - quat_IB.toRotationMatrix() * (pos_BE));
}

/* Callback Functions */
void Planner::_timer_callback()
{
    /* Init the Message and time stamp it */
    geometry_msgs::msg::PoseStamped msg{};
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();

    /* Extract current state */
    geometry_msgs::msg::PoseStamped curr_pos = this->_curr_pose;
    std::vector<double> joint_pos = this->_curr_js.position;

    /* Put in the position of the planner */
    Eigen::Vector3d position = this->get_trajectory_setpoint();

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
        // float curr_yaw = common::yaw_from_quaternion(
        Eigen::Quaterniond current_quat(curr_pos.pose.orientation.w,
                                        curr_pos.pose.orientation.x,
                                        curr_pos.pose.orientation.y,
                                        curr_pos.pose.orientation.z);
        Eigen::Vector3d aligned_position;

        Eigen::Quaterniond output_q;
        /* Find the aligned position and orientation */
        _align_to_wall(output_q, aligned_position, position,
                       this->_ee_offset + this->_curr_js.position[0] * Eigen::Vector3d::UnitY(),
                       this->_curr_js.position[1],
                       current_quat);

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
        RCLCPP_DEBUG(this->get_logger(), "%f %f %f", position.x(), position.y(), position.z());

        /* Transform to start point */
        auto curr_position = this->_curr_pose.pose.position;
        Eigen::Vector3d dir = (this->_start_point - Eigen::Vector3d(curr_position.x,
                                                                    curr_position.y,
                                                                    curr_position.z))
                                  .normalized();
        double dt = 1.0 / this->_frequency;
        position += this->_v_approach * dt * dir;

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
    this->_curr_pose = *msg;
}
void Planner::_ee_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->_curr_ee_pose = *msg;
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

    return force_over_threshold || trackball_over_threshold;
}