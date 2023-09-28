#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

namespace kinematics
{
    void forward_kinematics(Eigen::Matrix3d R_IB, Eigen::Vector3d p_IB, double joint_state[2], double imu_roll);
}

#endif // KINEMATICS_H