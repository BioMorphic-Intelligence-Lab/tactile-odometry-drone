#include "rclcpp/rclcpp.hpp"
#include "controller.hpp"

class TestTrajectoryPublisher : public Controller
{
public:

    /**
     * @brief Publish a trajectory setpoint
     */
    void publish_trajectory_setpoint() override;

};


