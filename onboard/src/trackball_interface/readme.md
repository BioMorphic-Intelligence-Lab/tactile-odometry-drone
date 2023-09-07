# Trackball Interface
## General Description
ROS2-Node to access data from trackball via USB like a mouse.

## Published topics
/trackball/position (geometry_msgs::msg::PointStamped)
integrated position since last reset. Published with constant frequenzy (20 Hz)

/trackball/ticks (geometry_msgs::msg::PointStamped)
latest sensor increment. Published when new data are available

## Provided Services
trackball_interface/setZero (std_srvs::srv::Trigger)
reset integrated position in topic /trackball/position



