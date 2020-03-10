/*
** feature.h for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Aug 6 下午3:24:38 2019 little fang
** Last update Wed Feb 11 下午2:17:13 2020 little fang
*/

#ifndef FEATURE_H_
#define FEATURE_H_

#include "msckf.hpp"
#include <map>
#include <iomanip>
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
    std::map<StateId, cv::Point2f> raw_uv_;
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

/**
  * @brief  override the ostream output map<key,value>
  * @note   
  * @param  &output: 
  * @param  &data: output data map<key,value>
  * @retval 
  */
template <typename key, typename value>
std::ostream &
operator<<(std::ostream &output, const std::map<key, value> &data)
{
    for (auto index : data)
    {
        output << "{" << index.first << ",[" << index.second << "]}  ";
    }
    return output;
};

std::ostream &operator<<(std::ostream &output, const CameraState &camera_state)
{
    output << std::setw(9) << camera_state.state_id_ << "  ";
    output << std::fixed << std::setprecision(4) << camera_state.position_.transpose() << "    ";
    output << std::fixed << std::setprecision(8) << camera_state.quat_.coeffs().transpose();
    return output;
};

std::ostream &operator<<(std::ostream &output, const Feature &feature)
{
    output << std::setw(9) << feature.feature_id_ << "  ";
    output << std::fixed << std::setprecision(8) << feature.observation_uv_ << "    ";
    output << std::fixed << std::setprecision(4) << feature.raw_uv_;
    return output;
};

} // namespace camera

} // namespace mscnav

#endif /* !FEATURE_H_ */
