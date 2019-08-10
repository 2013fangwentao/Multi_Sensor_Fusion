/*
** camera.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Camera State, 每一帧中记录当前的Camera对应的状态，位姿
**
** Started on  Tue Aug 6 下午3:19:51 2019 little fang
** Last update Sun Aug 10 下午2:31:36 2019 little fang
*/

#ifndef FRAME_H_
#define FRAME_H_

#include <map>
#include "navtime.h"
#include "navconfig.hpp"
#include "filter/navfilter.h"
#include "process/navstate.h"
#include "feature.hpp"
#include "imageprocess.h"
#include "Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace mscnav
{
namespace camera
{
class MsckfProcess
{
public:
    MsckfProcess(const KalmanFilter::Ptr &filter);
    ~MsckfProcess();

    bool ProcessImage(const cv::Mat &img1, Eigen::VectorXd &dx);

private:
    void FirstImageProcess(const cv::Mat &img1);
    void AguementedState();
    void AddObservation();
    void DetermineMeasureFeature();
    bool CheckEnableTriangleate(const Feature &feature);

    void FeatureJacobian();
    void MeasurementJacobian();
    void MeasurementUpdate();
    void RemoveLostFeature();
    void FindRedundantCamStates();

private:
    std::vector<cv::KeyPoint>
        pre_frame_keypoints_;
    std::vector<cv::KeyPoint> curr_frame_keypoints_;
    std::vector<cv::DMatch> matches_;
    cv::Mat pre_frame_descriptors_;
    cv::Mat curr_frame_descriptors_;
    Eigen::Isometry3d cam_imu_tranformation_;

    std::map<FeatureId, Feature> map_feature_set_;
    std::map<FeatureId, Feature> map_observation_set_;
    std::map<StateId, CameraState> map_state_set_;

private:
    std::map<int, FeatureId> trainidx_feature_map_;
    KalmanFilter::Ptr filter_;
    utiltool::ConfigInfo::Ptr config_;
    State::Ptr state_;
    bool is_first_ = true;
};
} // namespace camera
} // namespace mscnav

#endif /* !FRAME_H_ */
