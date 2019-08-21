/*
** camera.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Camera State, 每一帧中记录当前的Camera对应的状态，位姿
**
** Started on  Tue Aug 6 下午3:19:51 2019 little fang
** Last update Thu Aug 21 下午9:06:59 2019 little fang
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
#include <ceres/ceres.h>

namespace mscnav
{
namespace camera
{

struct ReprojectionError
{
    ReprojectionError(const cv::Point2f &measure_point_uv,
                      const Eigen::Isometry3d &cam0_cami_transform) : image_uv(measure_point_uv),
                                                                      cam0_cami(cam0_cami_transform) {}
    template <typename T>
    bool operator()(const T *const feature_point_world, T *residuals) const
    {
        Eigen::Matrix<T, 3, 1> point;
        point << feature_point_world[0], feature_point_world[1], feature_point_world[2];

        point = cam0_cami * point;
        T point_reproject_image[2] = {point(0) / point(2), point(1) / point(2)};
        residuals[0] = point_reproject_image[0] - T(image_uv.x);
        residuals[1] = point_reproject_image[1] - T(image_uv.y);
        return true;
    }

    static ceres::CostFunction *Create(const cv::Point2f &measure_point_uv,
                                       const Eigen::Isometry3d &cam0_cami_transform)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(new ReprojectionError(measure_point_uv, cam0_cami_transform));
    }

    const cv::Point2f image_uv;
    const Eigen::Isometry3d cam0_cami;
};

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
