/*
** feature.h for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Aug 6 下午3:24:38 2019 little fang
** Last update Fri Aug 8 上午10:50:03 2019 little fang
*/

#ifndef FEATURE_H_
#define FEATURE_H_

#include <map>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "camera.hpp"

namespace mscnav
{
namespace camera
{

class Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FeatureId = long long int;

public:
    Feature(const FeatureId feature_id) : feature_id_(feature_id) {}
    Feature() : feature_id_(for_next_one) { for_next_one++; }
    ~Feature();

private:
    static FeatureId for_next_one;

public:
    FeatureId feature_id_;
    Eigen::Vector3d position_world_{0, 0, 0}; //特征点的世界坐标
    std::map<CameraState::StateId, cv::KeyPoint> potition_camera_map_;
    bool is_initialized_ = false;
};

} // namespace camera

} // namespace mscnav

#endif /* !FEATURE_H_ */
