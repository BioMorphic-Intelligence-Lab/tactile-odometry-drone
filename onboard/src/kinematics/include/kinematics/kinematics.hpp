#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

namespace kinematics
{

    void forward_kinematics(Eigen::Matrix3d R_IB,
                        Eigen::Vector3d p_IB,
                        double joint_state[2],
                        double imu_roll,
                        Eigen::Matrix3d &R_IE,
                        Eigen::Matrix3d &R_IT,
                        Eigen::Matrix3d &R_IO,
                        Eigen::Matrix3d &R_IW,
                        Eigen::Vector3d &p_IE,
                        Eigen::Vector3d &p_IT,
                        Eigen::Vector3d &p_IO);

    void forward_kinematics(Eigen::Matrix3d R_IB,
                        Eigen::Vector3d p_IB,
                        double joint_state[2],
                        double imu_roll,
                        Eigen::Matrix3d &R_IO,
                        Eigen::Vector3d &p_IO);

}

#endif // KINEMATICS_H