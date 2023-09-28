#include "kinematics.hpp"
#include "common/common.hpp"

using namespace personal;

namespace kinematics
{
    const double normal_spring_length = 0.1;
    const Eigen::Vector3d p_TO = common::rot_z(M_PI) * Eigen::Vector3d(-0.01875, 0.0423, 0.0); // aka ee_to_ball_offset
    const Eigen::Vector3d p_BE = -Eigen::Vector3d(-0.03, 0.33, 0.046);                         // aka ee_to_base_offset normal_spring_length = 0.1;
    const Eigen::Vector3d p_ET_0 = Eigen::Vector3d(0, -normal_spring_length, 0);               // todo to be replaced

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
                            Eigen::Vector3d &p_IO)
    {
        /*convention for vector names:
           variable name: p
            indices: first: start frame, second: end frame
        expressed in frame: first index, if not specified otherwise */
        Eigen::Vector3d uav_eul = R_IB.eulerAngles(2, 1, 0);
        double uav_roll = uav_eul.y();

        R_IE = R_IB;
        p_IE = p_IB + R_IB * p_BE;
        Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state
        p_IT = p_IE + R_IE * p_ET;
        Eigen::Matrix3d R_ET = common::rot_z(joint_state[1]);
        R_IT = R_IE * R_ET;
        Eigen::Matrix3d R_TO = common::rot_x(M_PI / 2) * common::rot_z(-M_PI / 2) * common::rot_z(uav_roll - imu_roll);
        R_IO = R_IT * R_TO;
        p_IO = p_IT + R_IT * p_TO;

        double yaw_wall = common::yaw_from_quaternion_y_align(Eigen::Quaterniond(R_IO));

        // derive R_IW not from R_IO but only consider yaw from R_IO
        R_IW = common::rot_z(-(yaw_wall + M_PI / 2));
        // Eigen::Matrix3d R_OW = R_IO.transpose() * R_IW;
    }

}