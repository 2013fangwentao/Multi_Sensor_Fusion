/*
** navmech.cc for mscnav in /media/fwt/Data/program/mscnav/src/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Sat Jul 13 下午10:53:27 2019 little fang
** Last update Sat Mar 13 下午2:10:30 2020 little fang
*/

#include "imu/navmech.h"
#include "navconfig.hpp"
#include "navattitude.hpp"
#include "navearth.hpp"

using namespace utiltool;

namespace mscnav
{
namespace navmech
{
/**
 * @brief  完成全部的机械编排功能,并计算该时间段内的状态矩阵
 * @note   
 * @param  &pre_imu_data: 前一时刻的IMU观测值
 * @param  &curr_imu_data: 当前时刻的IMU观测值
 * @param  &pre_nav_info: 已有的最新的导航状态
 * @param  &curr_nav_info: 需要推算到的导航状态
 * @param  &phi_mat: 状态转移矩阵,离散化后的
 * @retval 推算到的导航状态,
 */
NavInfo &MechanicalArrangement(const ImuData &pre_imu_data, const ImuData &curr_imu_data, NavInfo &nav_info, Eigen::MatrixXd &phi_mat)
{
    nav_info_ = nav_info;
    nav_info_.quat_ = MechAttitudeUpdate(pre_imu_data, curr_imu_data, nav_info);
    nav_info_.vel_ = MechVelocityUpdate(pre_imu_data, curr_imu_data, nav_info);
    nav_info_.pos_ = MechPositionUpdate(nav_info, curr_imu_data.get_time() - pre_imu_data.get_time());
    nav_info_.time_ = curr_imu_data.get_time();
    phi_mat = MechTransferMat(pre_imu_data, curr_imu_data, nav_info_);
    // phi_mat = ModifyPhi(phi_mat, nav_info, nav_info_);
    NormalizeAttitude(nav_info_);
    nav_info = nav_info_;
    return nav_info;
}
namespace
{

Eigen::Quaterniond MechAttitudeUpdate(const ImuData &pre_imu_data, const ImuData &curr_imu_data, const NavInfo &pre_nav_info)
{
    using Eigen::Quaterniond;
    using Eigen::Vector3d;
    double dt = curr_imu_data.get_time() - pre_imu_data.get_time();
    Vector3d thet_k(curr_imu_data.gyro_), thet_k_1(pre_imu_data.gyro_);

    Vector3d curr_phi = thet_k + utiltool::skew(thet_k_1) * (thet_k) / 12.0;
    Quaterniond quat_bb = attitude::RotationVector2Quaternion(curr_phi);

    Vector3d zeta = earth::wiee() * dt;
    Quaterniond quat_ee = attitude::RotationVector2Quaternion(zeta).conjugate();
    Quaterniond qbn_k = quat_ee * (pre_nav_info.quat_ * quat_bb);

    /*赋值角速度和加速度 */
    nav_info_.wibb_ = Vector3d(curr_imu_data.gyro_ / dt);
    nav_info_.fibb_ = Vector3d(curr_imu_data.acce_ / dt);

    return qbn_k.normalized();
}

Eigen::Vector3d MechVelocityUpdate(const ImuData &pre_imu_data, const ImuData &curr_imu_data, const NavInfo &pre_nav_info)
{
    using Eigen::Matrix3d;
    using Eigen::Vector3d;
    auto &pos = pre_nav_info.pos_, &vel = pre_nav_info.vel_;
    double dt = curr_imu_data.get_time() - pre_imu_data.get_time();
    auto wiee = earth::wiee();
    Vector3d thet_k(curr_imu_data.gyro_), thet_k_1(pre_imu_data.gyro_);
    Vector3d vb_k(curr_imu_data.acce_), vb_k_1(pre_imu_data.acce_);

    auto BLH = earth::WGS84XYZ2BLH(pre_nav_info.pos_);
    auto Cne = earth::CalCn2e(BLH[0], BLH[1]);
    Vector3d ge_vec = Cne * earth::CalculateGravity(BLH, false);
    Vector3d omega_n = wiee * 2.0;
    Vector3d delta_gcor = (ge_vec - omega_n.cross(vel)) * dt;

    Matrix3d C_ee = Matrix3d::Identity() - utiltool::skew(wiee * 0.5 * dt);

    Vector3d vrot = thet_k.cross(vb_k) * 0.5;
    Vector3d vscul = (thet_k_1.cross(vb_k) + vb_k_1.cross(thet_k)) / 12.0;

    Matrix3d Cbe = pre_nav_info.quat_.toRotationMatrix();
    Vector3d delta_ve = C_ee * pre_nav_info.quat_.toRotationMatrix() * (vb_k + vrot + vscul);
    return (pre_nav_info.vel_ + delta_gcor + delta_ve);
}

Eigen::Vector3d MechPositionUpdate(const NavInfo &pre_nav_info, double dt)
{
    Eigen::Vector3d pos;
    pos = dt * (pre_nav_info.vel_ + nav_info_.vel_) * 0.5 + pre_nav_info.pos_;
    return pos;
}

/**
 * @brief  获取状态转移矩阵,排列顺序为 0 位置 3 速度 6 姿态角误差 9 陀螺零偏 12 加速度计零偏 15 陀螺比例因子 18 加速度计比例因子
 * @note
 * @param  &pre_imu_data:
 * @param  &curr_imu_data:
 * @param  nav_info:
 * @retval
 */
Eigen::MatrixXd MechTransferMat(const ImuData &pre_imu_data, const ImuData &curr_imu_data, const NavInfo &nav_info)
{
    using constant::constant_hour;
    using Eigen::Matrix3d;
    using Eigen::MatrixXd;
    using Eigen::Vector3d;
    utiltool::ConfigInfo::Ptr config = utiltool::ConfigInfo::GetInstance();

    static int scale_of_acce = config->get<int>("evaluate_imu_scale") == 0 ? 0 : 3;
    static int scale_of_gyro = scale_of_acce;
    static int camera_imu_rotation = config->get<int>("evaluate_camera_imu_rotation") == 0 ? 0 : 3;
    static int rows = 15 + scale_of_acce + scale_of_gyro + camera_imu_rotation, cols = rows;
    static double corr_time_of_gyro_bias = config->get<double>("corr_time_of_gyro_bias") * constant_hour;
    static double corr_time_of_acce_bias = config->get<double>("corr_time_of_acce_bias") * constant_hour;
    static double corr_time_of_gyro_scale = 0, corr_time_of_acce_scale = 0;
    if (scale_of_gyro == 3)
        corr_time_of_gyro_scale = config->get<double>("corr_time_of_gyro_scale") * constant_hour;
    if (scale_of_acce == 3)
        corr_time_of_acce_scale = config->get<double>("corr_time_of_acce_scale") * constant_hour;

    auto &pos = nav_info.pos_;
    auto &vel = nav_info.vel_;
    auto &fb = nav_info.fibb_;
    auto &wb = nav_info.wibb_;
    auto wiee = earth::wiee();
    auto Cbe = nav_info.quat_.toRotationMatrix();
    double dt = curr_imu_data.get_time() - pre_imu_data.get_time();

    MatrixXd F = MatrixXd::Zero(rows, cols);

    //位置对应的误差方程
    F.block<3, 3>(0, 3) = Matrix3d::Identity();

    //速度对应的误差方程
    // DONE 初步核实完毕,需要再核实
    // F.block<3, 3>(3, 3) = utiltool::skew(wiee) * -2.0;
    F.block<3, 3>(3, 6) = utiltool::skew(Cbe * fb);
    F.block<3, 3>(3, 12) = Cbe;
    if (scale_of_acce == 3)
    {
        F.block<3, 3>(3, 18) = Cbe * (fb.asDiagonal());
        F.block<3, 3>(18, 18) = Matrix3d::Identity() * (-1.0 / corr_time_of_acce_scale);
    }
    //姿态对应的误差方程
    // F.block<3, 3>(6, 6) = -1 * utiltool::skew(wiee);
    F.block<3, 3>(6, 9) = -Cbe;
    if (scale_of_gyro == 3)
    {
        F.block<3, 3>(6, 15) = -Cbe * (wb.asDiagonal());
        F.block<3, 3>(15, 15) = Matrix3d::Identity() * (-1.0 / corr_time_of_gyro_scale);
    }

    //IMU参数
    F.block<3, 3>(9, 9) = Matrix3d::Identity() * (-1 / corr_time_of_gyro_bias);
    F.block<3, 3>(12, 12) = Matrix3d::Identity() * (-1 / corr_time_of_acce_bias);
    return MatrixXd::Identity(rows, cols) + F * dt + 0.5 * (F * dt) * (F * dt);
}

/**
 * @brief  可观测性约束调整
 * @note 
 *  Modify the transition matrix
 *  For observility constrain
 *  OC-EKF: <<On the consistency of Vision-aided Inertial Navigation>> ref.1
 *  <Consistency Analysis and Improvement of Vision-aided Inertial Navigation> ref.2
 * @param  &phi: 
 * @param  &pre_info: 
 * @param  &curr_info: 
 * @retval None
 */
Eigen::MatrixXd &ModifyPhi(Eigen::MatrixXd &phi,
                           const NavInfo &pre_info,
                           const NavInfo &curr_info)
{
    double dt = curr_info.time_ - pre_info.time_;
    auto gravity = earth::CalculateGravity(pre_info.pos_);
    Eigen::Matrix3d R_kk_1 = curr_info.rotation_.transpose() * pre_info.rotation_;
    phi.block<3, 3>(6, 6) = R_kk_1;
    Eigen::Vector3d u = pre_info.rotation_.transpose() * gravity;
    Eigen::RowVector3d s = (u.transpose() * u).inverse() * u.transpose();
    Eigen::Matrix3d A1 = phi.block<3, 3>(3, 6);
    Eigen::Vector3d w1 = utiltool::skew(pre_info.vel_ - curr_info.vel_) * gravity;
    phi.block<3, 3>(3, 6) = A1 - (A1 * u - w1) * s;
    Eigen::Matrix3d A2 = phi.block<3, 3>(0, 6);
    Eigen::Vector3d w2 = utiltool::skew(dt * pre_info.vel_ + pre_info.pos_ - curr_info.pos_) * gravity;
    phi.block<3, 3>(0, 6) = A2 - (A2 * u - w2) * s;
    return phi;
}

} // namespace
} // namespace navmech

ImuData &Compensate(ImuData &curr_imu, const NavInfo &nav_info, const double dt)
{
    for (int i = 0; i < 3; i++)
    {
        curr_imu.acce_(i) = (curr_imu.acce_(i) - nav_info.acce_bias_(i) * dt) * (1 - nav_info.acce_scale_(i));
        curr_imu.gyro_(i) = (curr_imu.gyro_(i) - nav_info.gyro_bias_(i) * dt) * (1 - nav_info.gyro_scale_(i));
    }
    return curr_imu;
}
} // namespace mscnav
