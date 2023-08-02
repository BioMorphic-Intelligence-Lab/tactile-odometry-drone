#include "force_estimator.hpp"

ForceEstimatorNode::ForceEstimatorNode()
      : Node("force_estimator"),
        _NAMES{"linear_joint", "revolute_joint"}
{

  /* Declare all the parameters */
  // Arm properties
  this->declare_parameter("k_lin", 1.0);
  this->declare_parameter("k_rot", 0.001);
  this->declare_parameter("d_ee", 0.01);
  /* Get all the arm property parameters */
  this->_k_lin = this->get_parameter("k_lin").as_double();
  this->_k_rot = this->get_parameter("k_rot").as_double();
  this->_d_ee = this->get_parameter("d_ee").as_double();

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
  double f_spring = this->_k_lin * pos[0];
  /* Compute the torque required to bring rotational 
   * joint into its current position */
  double torque = this->_k_rot * pos[1];
  /* Given the torque we can find the lateral
   * component of the force*/
  double f_lat = - torque / this->_d_ee;

  /* Define the wrench message. The force is published in ee frame*/
  auto wrench_msg = geometry_msgs::msg::WrenchStamped();
  wrench_msg.header.stamp = msg->header.stamp;
  // By defining the force in the ee-frame it is easy to add
  // the lateral component
  wrench_msg.header.frame_id = "ee";

  /* In ee-frame the force is always normal, i.e. fx = 0*/
  wrench_msg.wrench.force.x = 0.0;
  wrench_msg.wrench.force.y = f_spring * sin(pos[1]) + f_lat;
  /* The normal force is found via the trigonometric relation through the rotational angle */
  wrench_msg.wrench.force.z = f_spring * cos(pos[1]);

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
