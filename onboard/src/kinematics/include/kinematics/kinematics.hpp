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

    void inverse_kinematics(Eigen::Matrix3d R_IO_0, Eigen::Vector3d p_IO_0, Eigen::Vector3d p_WO,
                            double joint_state[2], double joint_state_start[2], double imu_roll, double uav_roll,
                            Eigen::Matrix3d &R_IB,
                            Eigen::Matrix3d &R_IE,
                            Eigen::Matrix3d &R_IT,
                            Eigen::Matrix3d &R_IO,
                            Eigen::Matrix3d &R_IW,
                            Eigen::Vector3d &p_IE,
                            Eigen::Vector3d &p_IT,
                            Eigen::Vector3d &p_IO,
                            Eigen::Vector3d &p_IB);

    
    void inverse_kinematics(Eigen::Matrix3d R_IO_0, Eigen::Vector3d p_IO_0, Eigen::Vector3d p_WO,
                            double joint_state[2], double joint_state_start[2], double imu_roll, double uav_roll,
                            Eigen::Matrix3d &R_IB,
                            Eigen::Vector3d &p_IB);

}

#endif // KINEMATICS_H