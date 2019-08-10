/*
** feature.h for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Aug 6 下午3:24:38 2019 little fang
** Last update Sun Aug 10 下午2:33:37 2019 little fang
*/

#ifndef FEATURE_H_
#define FEATURE_H_

#include <map>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace mscnav
{
namespace camera
{
class CameraState;

typedef long long int FeatureId;
typedef long long int StateId;

class Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Feature(const FeatureId feature_id) : feature_id_(feature_id) {}
    Feature() : feature_id_(for_next_one) { for_next_one++; }
    ~Feature() {}

private:
    static FeatureId for_next_one;

public:
    FeatureId feature_id_;
    Eigen::Vector3d position_world_{0, 0, 0}; //特征点的世界坐标
    std::map<StateId, cv::Point2f> observation_uv_;
    bool is_initialized_ = false;
};
FeatureId Feature::for_next_one(0);

class CameraState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    CameraState(const StateId state_id) : state_id_(state_id) {}
    CameraState() : state_id_(for_next_one) { for_next_one++; }
    ~CameraState() {}

public:
    Eigen::Quaterniond quat_{1, 0, 0, 0};
    Eigen::Vector3d position_{0, 0, 0};
    StateId state_id_ = 0;
    utiltool::NavTime time_;
    std::vector<FeatureId> feature_id_set_;

private:
    static StateId for_next_one;
};
StateId CameraState::for_next_one(0);
} // namespace camera

} // namespace mscnav

#endif /* !FEATURE_H_ */
