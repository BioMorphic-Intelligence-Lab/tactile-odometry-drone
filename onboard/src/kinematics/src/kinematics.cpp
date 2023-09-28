#include "kinematics.hpp"

namespace personal
{
    namespace kinematics
    {
        void forward_kinematics(Eigen::Matrix3d R_IB, Eigen::Vector3d p_IB, double joint_state[2], double imu_roll)
        {
            /*convention for vector names:
               variable name: p
                indices: first: start frame, second: end frame
            expressed in frame: first index, if not specified otherwise */

            Eigen::Vector3d p_TO = common::rot_z(M_PI) * Eigen::Vector3d(-0.01875, 0.0423, 0.0); // aka ee_to_ball_offset
            Eigen::Vector3d p_BE = -Eigen::Vector3d(-0.03, 0.33, 0.046);                         // aka ee_to_base_offset normal_spring_length = 0.1;
            Eigen::Vector3d p_ET_0 = Eigen::Vector3d(0; -normal_spring_length; 0);               // todo to be replaced
            Eigen::Vector3d uav_eul = R_IB.eulerAngles(2, 1, 0);
            double uav_roll = uav_eul.y();

            Eigen::Matrix3d R_IE = R_IB;
            Eigen::Vector3d p_IE = p_IB + R_IB * p_BE;
            Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state
            Eigen::Vector3d p_IT = p_IE + R_IE * p_ET;
            Eigen::Matrix3d R_ET = common::rot_z(joint_state(2));
            Eigen::Matrix3d R_IT = R_IE * R_ET;
            Eigen::Matrix3d R_TO = common::rot_x(M_PI / 2) * common::rot_z(-M_PI / 2) * common::rot_z(uav_roll - imu_roll);
            Eigen::Matrix3d R_IO = R_IT * R_TO;
            // p_TO = ee_to_ball_offset;
            Eigen::Vector3d p_IO = p_IT + R_IT * p_TO;

            yaw = atan2d(R_IO(1, 2), R_IO(2, 2));

            // derive R_IW not from R_IO but only consider yaw from R_IO
            Eigen::Matrix3d R_IW = common_rot_z(-(yaw + M_PI / 2));
            Eigen::Matrix3d R_OW = R_IO.transpose() * R_IW;
        }
    }
}