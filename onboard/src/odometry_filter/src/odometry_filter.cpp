#include "odometry_filter.hpp"
using namespace std::chrono_literals;

OdometryFilter::OdometryFilter()
    : Node("odometry_filter")
{
  printf("constructor...");

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
      "/onboard/pose_estimate",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryFilter::_state_callback, this, std::placeholders::_1));

  /* Init Publishers */
  this->odom_pose_wall_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/odometry/pose_wall", 10);
  this->odom_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/odometry/pose", 10);
  this->_roll_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/odometry/roll", 10);
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
  printf("starting...");
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
  this->R_BW = personal::common::rot_z(-this->encoder_yaw);                                                        // ToDo: check signs
  this->R_IB = personal::common::rot_z(this->yaw_at_contact - (this->encoder_yaw - this->encoder_yaw_at_contact)); // ToDo: check signs and unite R_IB*R_BW in one single, minimlal rot_z

  Eigen::Vector3d pos_B = this->R_BW * (pos_wall - this->ee_to_ball_offset);
  pos_B = pos_B + this->ee_to_base_offset;
  pos_B.y() -= this->linear_joint; // TODO Check sign
  Eigen::Vector3d pos_world = this->R_IB * pos_B + this->pos_at_contact;

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

  Eigen::Vector3d pos_wall = this->odom_to_wall(this->_trackball_pos, this->quat_imu);

  Eigen::Vector3d pos_world = this->wall_to_world(pos_wall);

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

  this->odom_pose_wall_publisher_->publish(pose_wall_msg);
  this->odom_pose_publisher_->publish(pose_msg);
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
  this->linear_joint = msg;
}

void OdometryFilter::_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  this->in_contact = msg->data;
  this->evaluate_contact();
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
