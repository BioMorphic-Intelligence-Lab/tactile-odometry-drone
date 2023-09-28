#include "kinematics.hpp"
#include "common/common.hpp"

using namespace personal;

namespace kinematics
{
    const Eigen::Vector3d p_TO = Eigen::Vector3d(0.0169, -0.042, -0.02);
    const Eigen::Vector3d p_BE = -Eigen::Vector3d(-0.03, 0.33, 0.046);                         
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
                            Eigen::Matrix3d &R_IW,
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
        Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state
        p_IT = p_IE + R_IE * p_ET;
        Eigen::Matrix3d R_ET = common::rot_z(-joint_state[1]);
        R_IT = R_IE * R_ET;
        Eigen::Matrix3d R_TO = common::rot_x(M_PI / 2) * common::rot_z(-M_PI / 2) * common::rot_z(uav_roll - imu_roll);
        R_IO = R_IT * R_TO;
        p_IO = p_IT + R_IT * p_TO;

        double yaw_wall = common::yaw_from_quaternion_y_align(Eigen::Quaterniond(R_IO));

        // derive R_IW not from R_IO but only consider yaw from R_IO
        R_IW = common::rot_z((yaw_wall - M_PI / 2));
        // Eigen::Matrix3d R_OW = R_IO.transpose() * R_IW;
    }

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
                            Eigen::Vector3d &p_IB)
    // inverse_kinematics aka wall_to_world
    // p_WO aka pos_wall
    // the origin of the wall frame is at the point of first contact: p_IW = p_IO_0
    // the orientaion of the wall frame is parallel to the orientation of the
    // odometry-frame at contactn (with roll=0): R_IW = R_IO_0*R_OW;
    {
        R_IO = R_IO_0 * common::rot_z(joint_state[1] - joint_state_start[1]);
        double yaw = common::yaw_from_quaternion_y_align(Eigen::Quaterniond(R_IO));

        // R_IW = R_IO*R_OW; //R_IW nicht aus R_IO ableiten, sondern nur yaw aus R_IO nehmen
        R_IW = common::rot_z(-(yaw + M_PI / 2));
        Eigen::Matrix3d R_OW = R_IO.transpose() * R_IW;

        Eigen::Matrix3d R_TO = common::rot_x(M_PI / 2) * common::rot_z(-M_PI / 2) * common::rot_z(uav_roll - imu_roll);
        Eigen::Matrix3d R_OT = R_TO.transpose();

        Eigen::Matrix3d R_ET = common::rot_z(-joint_state[1]);
        Eigen::Matrix3d R_TE = R_ET.transpose();
        Eigen::Matrix3d R_EB = Eigen::Matrix3d::Identity();

        R_IT = R_IO * R_OT;
        R_IE = R_IT * R_TE;
        R_IB = R_IE * R_EB;

        Eigen::Vector3d p_ET = p_ET_0 - Eigen::Vector3d(0, joint_state[0], 0); // compression of spring is negative joint_state

        p_IO = p_IO_0 + R_IO * R_OW * p_WO;
        p_IT = p_IO - R_IT * p_TO;
        p_IE = p_IT - R_IE * p_ET;
        p_IB = p_IE - R_IB * p_BE;
    }

    void inverse_kinematics(Eigen::Matrix3d R_IO_0, Eigen::Vector3d p_IO_0, Eigen::Vector3d p_WO,
                            double joint_state[2], double joint_state_start[2], double imu_roll, double uav_roll,
                            Eigen::Matrix3d &R_IB,
                            Eigen::Vector3d &p_IB)
    {
        Eigen::Matrix3d R_IE, R_IT, R_IO, R_IW;
        Eigen::Vector3d p_IE, p_IT, p_IO;
        
        inverse_kinematics(R_IO_0, p_IO_0, p_WO, joint_state, joint_state_start, imu_roll, uav_roll,
                           R_IB, R_IE, R_IT, R_IO, R_IW, p_IE, p_IT, p_IO, p_IB);
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
                           R_IE, R_IT, R_IO, R_IW, p_IE, p_IT, p_IO);
    }

}