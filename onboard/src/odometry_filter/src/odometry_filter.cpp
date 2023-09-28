#include "odometry_filter.hpp"
using namespace std::chrono_literals;

OdometryFilter::OdometryFilter()
    : Node("odometry_filter")
{
  printf("constructor...");

  this->declare_parameter("pose_topic", "/mocap_pose");
  /* Init Timer and subscribers*/
  // this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
  //                                      std::bind(&OdometryFilter::_timer_callback, this));
  this->_trackball_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/trackballX19/position",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_trackball_callback, this, std::placeholders::_1));
  this->_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_imu_callback, this, std::placeholders::_1));
  this->_contact_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "/in_contact",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_contact_callback, this, std::placeholders::_1));
  this->_state_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("pose_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_state_callback, this, std::placeholders::_1));
  this->_joint_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_state",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_joint_callback, this, std::placeholders::_1));

  /* Init Publishers */
  this->odom_pose_wall_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/odometry/pose_wall", 10);
  this->odom_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/odometry/pose", 10);
  this->_roll_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/odometry/roll", 10);
  this->_yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/odometry/yaw", 10);
  /*service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("trackball_position", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&OdometryFilter::timer_callback, this), timer_cb_group_);

        zeroService =
            this->create_service<std_srvs::srv::Trigger>("odometry_filter/setZero", std::bind(&OdometryFilter::setToZeroCB, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
                                                         service_cb_group_);

        */
  this->R_EO = personal::common::rot_z(-M_PI / 2) * personal::common::rot_y(-M_PI / 2);
  this->R_WE_0 = personal::common::rot_y(M_PI);

  /* Init TF publisher */
  this->_tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  printf("starting...");
}

void OdometryFilter::_detect_contact()
{
  // const bool force_over_threshold = fabs(this->linear_joint) > JS_THRESHOLD;
  const bool trackball_over_threshold = this->_trackball_pos.norm() > 0.01;

  // return force_over_threshold || trackball_over_threshold;
  // return trackball_over_threshold;
  this->in_contact = trackball_over_threshold;

  this->evaluate_contact();
}

void OdometryFilter::evaluate_contact()
{
  if (this->in_contact && !this->in_contact_last) // positive edge
  {
    this->encoder_yaw_at_contact = this->encoder_yaw;
    this->yaw_at_contact = personal::common::yaw_from_quaternion_y_align(this->quat_moCap);
    this->pos_at_contact = this->pos_ekf2;
  }
  this->in_contact_last = this->in_contact;
}

Eigen::Vector3d OdometryFilter::wall_to_world(Eigen::Vector3d pos_wall)
{
  this->R_BW = personal::common::rot_z(-this->encoder_yaw);                                         // ToDo: check signs
  double yaw_IB = this->yaw_at_contact + (this->encoder_yaw - this->encoder_yaw_at_contact) - M_PI; // ToDo: check signs and unite R_IB*R_BW in one single, minimlal rot_z
  this->R_IB = personal::common::rot_z(yaw_IB);

  std_msgs::msg::Float64 yaw_msg;
  yaw_msg.data = yaw_IB;
  this->_yaw_publisher_->publish(yaw_msg);

  Eigen::Vector3d pos_B = this->R_BW * (pos_wall - this->ee_to_ball_offset);
  pos_B = pos_B + this->ee_to_base_offset;
  pos_B.y() += this->linear_joint; // TODO Check sign
  Eigen::Vector3d pos_world = this->R_IB * pos_B + this->pos_at_contact;
  if (this->in_contact)
  {
    std::cout << "pos_wall" << std::endl
              << pos_wall << std::endl;
    std::cout << "pos_B" << std::endl
              << pos_B << std::endl;
    std::cout << "pos_world" << std::endl
              << pos_world << std::endl;

    std::cout << "pos_at_contact" << std::endl
              << this->pos_at_contact << std::endl;
  }

  // shift position to body frame
  return pos_world;
  ;
}
Eigen::Vector3d OdometryFilter::odom_to_wall(Eigen::Vector3d pos_odom, Eigen::Quaterniond quat_imu)
{
  pos_odom.z() = 0.0;
  const double roll = personal::common::roll_from_quaternion(quat_imu);

  const Eigen::Matrix3d R_WE = personal::common::rot_y(-roll);

  Eigen::Vector3d pos_e = this->R_EO * (pos_odom - this->pos_odom_last);
  Eigen::Vector3d pos_wall = this->pos_wall_last + R_WE * this->R_WE_0 * pos_e;

  /*
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "pos_e" << std::endl
              << pos_e << std::endl;
    std::cout << "R_WE" << std::endl
              << R_WE.format(CleanFmt) << std::endl;
    std::cout << "R_WE_0" << std::endl
              << this->R_WE_0.format(CleanFmt) << std::endl;
    std::cout << "pos_wall" << std::endl
              << pos_wall << std::endl;
              */

  std_msgs::msg::Float64 roll_msg;

  roll_msg.data = roll;
  this->_roll_publisher_->publish(roll_msg);

  this->pos_odom_last = pos_odom;
  this->pos_wall_last = pos_wall;

  return pos_wall;
}

// #########################
// ####### Callbacks #######
// #########################

void OdometryFilter::_trackball_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  this->_trackball_pos.x() = msg->point.x;
  this->_trackball_pos.y() = msg->point.y;
  this->_trackball_pos.z() = msg->point.z;

  if (!this->is_init)
  {
    this->is_init = true;
    this->pos_odom_at_init = this->_trackball_pos;
  }
  this->_trackball_pos -= this->pos_odom_at_init;

  this->_detect_contact(); // TODO move to extra node

  Eigen::Vector3d pos_wall = this->odom_to_wall(this->_trackball_pos, this->quat_imu);

  double imu_roll = 0.0;
  double uav_roll = 0.0;
  Eigen::Matrix3d R_IE, R_IT, R_IO_0, R_IO, R_IW, R_IB_0;
  Eigen::Vector3d p_IE, p_IT, p_IO_0, p_IO, p_IB_0, pos_world;

  R_IB_0 = this->orientation_at_contact.toRotationMatrix();
  p_IB_0 = this->pos_at_contact;

  double joint_state[2] = {this->linear_joint, this->encoder_yaw};

  double joint_state_start[2] = {0.0, this->encoder_yaw_at_contact};
  // shift pos_wall, so that pos_IB is zero before contact
  kinematics::forward_kinematics(R_IB_0, p_IB_0, joint_state_start, imu_roll, uav_roll, R_IO_0, p_IO_0);

  kinematics::inverse_kinematics(R_IO_0,
                                 p_IO_0,
                                 pos_wall,
                                 joint_state,
                                 joint_state_start,
                                 imu_roll,
                                 uav_roll,
                                 R_IB, R_IE, R_IT, R_IO, R_IW, p_IE, p_IT, p_IO, pos_world);
  Eigen::Quaterniond quat_IB = Eigen::Quaterniond(R_IB);
  // Eigen::Vector3d pos_world = this->wall_to_world(pos_wall);

  this->_publish_tf(R_IB, pos_world, "Body_est");
  this->_publish_tf(R_IE, p_IE, "EE_est");
  this->_publish_tf(R_IT, p_IT, "TCP_est");
  this->_publish_tf(R_IO, p_IO, "Odom_est");
  this->_publish_tf(R_IW, p_IO, "Wall_est");

  /*Publish Data*/
  geometry_msgs::msg::PoseStamped pose_wall_msg;
  pose_wall_msg.header.stamp = this->now();
  pose_wall_msg.pose.position.x = pos_wall.x();
  pose_wall_msg.pose.position.y = pos_wall.y();
  pose_wall_msg.pose.position.z = pos_wall.z();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = pose_wall_msg.header.stamp; // use same stamp to be able to match messages
  pose_msg.pose.position.x = pos_world.x();
  pose_msg.pose.position.y = pos_world.y();
  pose_msg.pose.position.z = pos_world.z();
  pose_msg.pose.orientation.w = quat_IB.w();
  pose_msg.pose.orientation.x = quat_IB.x();
  pose_msg.pose.orientation.y = quat_IB.y();
  pose_msg.pose.orientation.z = quat_IB.z();

  this->odom_pose_wall_publisher_->publish(pose_wall_msg);
  this->odom_pose_publisher_->publish(pose_msg);
}

geometry_msgs::msg::Transform OdometryFilter::_transform_from_eigen(Eigen::Quaterniond rot, Eigen::Vector3d pos)
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

void OdometryFilter::_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  this->pos_ekf2.x() = msg->pose.position.x;
  this->pos_ekf2.y() = msg->pose.position.y;
  this->pos_ekf2.z() = msg->pose.position.z;

  this->quat_moCap.w() = msg->pose.orientation.w;
  this->quat_moCap.x() = msg->pose.orientation.x;
  this->quat_moCap.y() = msg->pose.orientation.y;
  this->quat_moCap.z() = msg->pose.orientation.z;
}

void OdometryFilter::_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  this->quat_imu.w() = msg->orientation.w;
  this->quat_imu.x() = msg->orientation.x;
  this->quat_imu.y() = msg->orientation.y;
  this->quat_imu.z() = msg->orientation.z;
}

void OdometryFilter::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  this->linear_joint = msg->position[0];
  this->encoder_yaw = msg->position[1];
}

void OdometryFilter::_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  this->in_contact = msg->data;
  this->evaluate_contact();
}

void OdometryFilter::_publish_tf(Eigen::Matrix3d R,
                                 Eigen::Vector3d p,
                                 std::string child_name)
{
  geometry_msgs::msg::TransformStamped T;
  T.header.stamp = this->now();
  T.header.frame_id = "world";
  T.child_frame_id = child_name;

  T.transform = this->_transform_from_eigen(Eigen::Quaterniond(R), p);
  this->_tf_broadcaster->sendTransform(T);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<OdometryFilter>();
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Starting odometry_filter node, shut down with CTRL-C");
  executor.spin();
  return 0;
}
