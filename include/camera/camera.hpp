/*
** camera.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Camera State, 每一帧中记录当前的Camera对应的状态，位姿
**
** Started on  Tue Aug 6 下午3:19:51 2019 little fang
** Last update Fri Aug 8 上午10:23:07 2019 little fang
*/

#ifndef CMAERA_H_
#define CMAERA_H_

#include "navtime.h"
#include "Eigen/Dense"
#include "opencv2/core/core.hpp"

namespace mscnav
{
namespace camera
{

class CameraState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using StateId = long long int;

public:
    CameraState(const StateId state_id) : state_id_(state_id) {}
    CameraState() : state_id_(for_next_one) { for_next_one++; }
    ~CameraState();

public:
    Eigen::Quaterniond quat_{1, 0, 0, 0};
    Eigen::Vector3d position_{0, 0, 0};
    StateId state_id_ = 0;
    utiltool::NavTime time_;

private:
    static StateId for_next_one;
};
CameraState::StateId CameraState::for_next_one(0);
} // namespace camera
} // namespace mscnav

#endif /* !CMAERA_H_ */
