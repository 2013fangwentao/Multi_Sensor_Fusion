/*
** navmech.h for mscnav in /media/fwt/Data/program/mscnav/include/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Sat Jul 13 下午10:32:43 2019 little fang
** Last update Sat Mar 13 下午1:42:03 2020 little fang
*/

#ifndef IMU_MECH_H_
#define IMU_MECH_H_

#include "navstruct.hpp"
#include <Eigen/Dense>

namespace mscnav
{
using utiltool::ImuData;
using utiltool::NavInfo;

ImuData &Compensate(ImuData &curr_imu, const NavInfo &nav_info, const double dt);

namespace navmech
{

NavInfo &MechanicalArrangement(const ImuData &pre_imu_data,
                               const ImuData &curr_imu_data,
                               NavInfo &nav_info,
                               Eigen::MatrixXd &phi_mat);
namespace
{
Eigen::Quaterniond MechAttitudeUpdate(const ImuData &pre_imu_data,
                                      const ImuData &curr_imu_data,
                                      const NavInfo &pre_nav_info);

Eigen::Vector3d MechVelocityUpdate(const ImuData &pre_imu_data,
                                   const ImuData &curr_imu_data,
                                   const NavInfo &pre_nav_info);

Eigen::Vector3d MechPositionUpdate(const NavInfo &pre_nav_info,
                                   double dt);

Eigen::MatrixXd MechTransferMat(const ImuData &pre_imu_data,
                                const ImuData &curr_imu_data,
                                const NavInfo &nav_info);

Eigen::MatrixXd &ModifyPhi(Eigen::MatrixXd &phi,
                           const NavInfo &pre_info,
                           const NavInfo &curr_info);

utiltool::NavInfo nav_info_;
}; // namespace
}; // namespace navmech

} // namespace mscnav

#endif /* !IMU_MECH_H_ */
