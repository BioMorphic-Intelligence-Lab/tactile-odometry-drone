#include "force_estimator.hpp"

ForceEstimatorNode::ForceEstimatorNode()
      : Node("force_estimator"),
        _NAMES{"linear_joint", "revolute_joint"}
{

  /* Declare all the parameters */
  // Arm properties
  this->declare_parameter("k", 0.1);
  /* Get all the arm property parameters */
  this->_k = this->get_parameter("k").as_double();

  this->_force = {0.0, 0.0, 0.0};

  /* Init all the class members */
  this->_force_publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);
  this->_joint_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(), std::bind(&ForceEstimatorNode::_joint_callback, this, std::placeholders::_1));
}

void ForceEstimatorNode::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  /* Read out the joint state values */
  double pos[2] = {msg->position[0],
                   msg->position[1]};

  /* Compute the force acting on the linear spring */
  double f_spring = this->_k * pos[0];

  /* Define the wrench message. The force is published in ee frame*/
  auto wrench_msg = geometry_msgs::msg::WrenchStamped();
  wrench_msg.header.stamp = msg->header.stamp;
  wrench_msg.header.frame_id = "ee";

  /* In ee-frame the force is always normal, i.e. fx = fy = 0*/
  wrench_msg.wrench.force.x = 0.0;
  wrench_msg.wrench.force.y = 0.0;
  /* The normal force is found via the trigonometric relation through the rotational angle */
  wrench_msg.wrench.force.z = f_spring / cos(pos[1]);

  /* Actually publish the message */
  this->_force_publisher->publish(wrench_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<ForceEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
