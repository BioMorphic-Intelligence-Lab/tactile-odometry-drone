#include "kinematics.hpp"
#include "common/common.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace personal;

namespace kinematics
{
    const Eigen::Vector3d p_TO = Eigen::Vector3d(-0.02044, -0.0423, -0.01688);
    const Eigen::Vector3d p_BE = -Eigen::Vector3d(-0.03, 0.33, 0.074);
    const Eigen::Vector3d p_ET_0 = Eigen::Vector3d(-0.03, -0.099, 0.007);

    // I = Inertial
    // B = Body
    // E = EE- marker
    // T = Tool-Center Point (Rotational Axis)
    // O = Odometry (Ball Tip)
    // W = Wall
    void forward_kinematics(Eigen::Matrix3d R_IB, // Micap Orientation
                            Eigen::Vector3d p_IB, // Mocap Position
                            double joint_state[2],
                            double imu_roll,
                            double uav_roll,
                            Eigen::Matrix3d &R_IE,
                            Eigen::Matrix3d &R_IT,
                            Eigen::Matrix3d &R_IO,
                            Eigen::Vector3d &p_IE,
                            Eigen::Vector3d &p_IT,
                            Eigen::Vector3d &p_IO)
    {
        /*convention for vector names:
           variable name: p
            indices: first: start frame, second: end frame
        expressed in frame: first index, if not specified otherwise */

        R_IE = R_IB;
        p_IE = p_IB + R_IB * p_BE;
        forward_kinematics_E_to_O(R_IE, p_IE, joint_state, imu_roll, uav_roll,
                                  R_IT, R_IO, p_IT, p_IO);
    }

    /*
    transform the psition of the odometry which is expressed w.r.t. the wall-frame to the world frame
    */
    Eigen::Vector3d transform_wall_to_world(Eigen::Vector3d p_IO_0, Eigen::Vector3d p_WO, Eigen::Matrix3d R_IW)
    {
        Eigen::Vector3d p_IO = p_IO_0 + R_IW * p_WO;

        return p_IO;
    }

    void inverse_kinematics(Eigen::Matrix3d R_IO, Eigen::Vector3d p_IO,
                            double joint_state[2], double imu_roll, double uav_roll,
                            Eigen::Matrix3d &R_IB,
                            Eigen::Matrix3d &R_IE,
                            Eigen::Matrix3d &R_IT,
                            Eigen::Matrix3d &R_IW,
                            Eigen::Vector3d &p_IE,
                            Eigen::Vector3d &p_IT,
                            Eigen::Vector3d &p_IB)
    {

        // the wall is assumed to be parallel to the odom-frame
        double wall_yaw = common::yaw_from_quaternion_y_align(Eigen::Quaterniond(R_IO));
        R_IW = common::rot_z(wall_yaw);

        Eigen::Matrix3d R_TO = common::rot_y(uav_roll - imu_roll);
        Eigen::Matrix3d R_OT = R_TO.transpose();

        Eigen::Matrix3d R_ET = common::rot_z(-joint_state[1]);
        Eigen::Matrix3d R_TE = R_ET.transpose();
        Eigen::Matrix3d R_EB = Eigen::Matrix3d::Identity();

        R_IT = R_IO * R_OT;
        R_IE = R_IT * R_TE;
        R_IB = R_IE * R_EB;

        Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state

        p_IT = p_IO - R_IT * p_TO;
        p_IE = p_IT - R_IE * p_ET;
        p_IB = p_IE - R_IB * p_BE;
    }

    void inverse_kinematics(Eigen::Matrix3d R_IO, Eigen::Vector3d p_IO,
                            double joint_state[2], double imu_roll, double uav_roll,
                            Eigen::Matrix3d &R_IB,
                            Eigen::Vector3d &p_IB)
    {
        Eigen::Matrix3d R_IE, R_IT, R_IW;
        Eigen::Vector3d p_IE, p_IT;

        inverse_kinematics(R_IO, p_IO, joint_state, imu_roll, uav_roll,
                           R_IB, R_IE, R_IT, R_IW, p_IE, p_IT, p_IB);
    }
    void forward_kinematics_E_to_O(Eigen::Matrix3d R_IE,
                                   Eigen::Vector3d p_IE,
                                   double joint_state[2],
                                   double imu_roll,
                                   double uav_roll,
                                   Eigen::Matrix3d &R_IT,
                                   Eigen::Matrix3d &R_IO,
                                   Eigen::Vector3d &p_IT,
                                   Eigen::Vector3d &p_IO)
    {
        Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state
        p_IT = p_IE + R_IE * p_ET;
        Eigen::Matrix3d R_ET = common::rot_z(-joint_state[1]);
        R_IT = R_IE * R_ET;
        Eigen::Matrix3d R_TO = common::rot_z(uav_roll - imu_roll);
        R_IO = R_IT * R_TO;
        p_IO = p_IT + R_IT * p_TO;
    }
    void forward_kinematics(Eigen::Matrix3d R_IB,
                            Eigen::Vector3d p_IB,
                            double joint_state[2],
                            double imu_roll,
                            double uav_roll,
                            Eigen::Matrix3d &R_IO,
                            Eigen::Vector3d &p_IO)
    {
        Eigen::Matrix3d R_IE, R_IT, R_IW;
        Eigen::Vector3d p_IE, p_IT;

        forward_kinematics(R_IB, p_IB, joint_state, imu_roll, uav_roll,
                           R_IE, R_IT, R_IO, p_IE, p_IT, p_IO);
    }

}