/*
** camera.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Camera State, 每一帧中记录当前的Camera对应的状态，位姿
**
** Started on  Tue Aug 6 下午3:19:51 2019 little fang
** Last update Wed Aug 13 下午3:30:46 2019 little fang
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

    bool ProcessImage(const cv::Mat &img1, const utiltool::NavTime &time);

private:
    void FirstImageProcess(const cv::Mat &img1);
    void AguementedState();
    void AddObservation();
    void DetermineMeasureFeature();
    bool CheckEnableTriangleate(const Feature &feature);
    bool LMOptimizatePosition(Feature &feature);

    bool MeasurementJacobian(const Feature &feature,
                             Eigen::MatrixXd &H_state,
                             Eigen::VectorXd &z_measure);
    Eigen::VectorXd MeasurementUpdate(const Eigen::MatrixXd &H_state,
                                      const Eigen::VectorXd &z_measure);

    void ReviseCameraState(const Eigen::VectorXd &dx_camera);

    void FeatureMeasureUpdate();

    void RemoveCameraState();
    void RemoveRedundantCamStates();

private:
    std::vector<cv::DMatch> matches_;
    cv::Mat pre_frame_descriptors_;
    cv::Mat curr_frame_descriptors_;
    Eigen::Isometry3d cam_imu_tranformation_;
    std::vector<cv::KeyPoint> pre_frame_keypoints_;
    std::vector<cv::KeyPoint> curr_frame_keypoints_;

    std::map<FeatureId, Feature> map_feature_set_;
    std::map<FeatureId, Feature> map_observation_set_;
    std::map<StateId, CameraState> map_state_set_;

private:
    bool is_first_ = true;
    double tracking_rate_ = 1.0;
    cv::Mat camera_mat_;
    cv::Mat dist_coeffs_;
    State::Ptr state_;
    KalmanFilter::Ptr filter_;
    utiltool::ConfigInfo::Ptr config_;
    std::map<int, FeatureId> trainidx_feature_map_;
    utiltool::NavTime curr_time_;
};
} // namespace camera
} // namespace mscnav

#endif /* !FRAME_H_ */
