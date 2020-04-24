/*
** camera.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Camera State, 每一帧中记录当前的Camera对应的状态，位姿
**
** Started on  Tue Aug 6 下午3:19:51 2019 little fang
** Last update Sat Mar 13 下午1:22:10 2020 little fang
*/

#ifndef MSCKFPROCESS_H_
#define MSCKFPROCESS_H_

#include "navtime.h"
#include "navconfig.hpp"
#include "filter/navfilter.h"
#include "navattitude.hpp"
#include "feature.hpp"
#include "imageprocess.h"
#include "Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <map>

namespace mscnav
{
namespace camera
{

class MsckfProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    using Ptr = std::shared_ptr<MsckfProcess>;

    MsckfProcess(const KalmanFilter::Ptr &filter);
    ~MsckfProcess() {}

    bool ProcessImage(const cv::Mat &img_raw, const utiltool::NavTime &time, utiltool::NavInfo &navinfo);
    void ReviseCameraState(const Eigen::VectorXd &dx_camera);

private:
    void FirstImageProcess(const cv::Mat &img1, const utiltool::NavInfo &navifo);
    void AguementedState(const utiltool::NavInfo &navinfo);
    void AddObservation();
    void DetermineMeasureFeature();
    bool CheckEnableTriangleate(const Feature &feature);
    bool CheckMotionStatus(const Feature &feature);
    bool LMOptimizatePosition(Feature &feature);
    bool TriangulatePoint(const Feature &feature, const Eigen::Isometry3d &cam0_trans, double position_cam0[3]);

    bool MeasurementJacobian(const Feature &feature,
                             Eigen::MatrixXd &H_state,
                             Eigen::VectorXd &z_measure);

    Eigen::VectorXd MeasurementUpdate(const Eigen::MatrixXd &H_state,
                                      const Eigen::VectorXd &z_measure);

    void FeatureMeasureUpdate(utiltool::NavInfo &navinfo);

    void RemoveCameraState();
    void RemoveRedundantCamStates(utiltool::NavInfo &navinfo);
    void RemoveRedundantCamStates();
    bool GatingTest(const Eigen::MatrixXd &H, const Eigen::VectorXd &r, const int &dof);

    void NormKeyPoints(const std::vector<cv::Point2f> &keypoint_distorted,
                       std::vector<cv::Point2f> &keypoint_undistorted,
                       const cv::Mat &camera_mat_);
    void Test();
    bool LMOptimizatePositionAndCheck(Feature &feature);
    bool CheckStaticMotion();

private:
    std::vector<cv::DMatch> matches_;
    cv::Mat pre_frame_descriptors_;
    cv::Mat curr_frame_descriptors_;
    Eigen::Isometry3d cam_imu_tranformation_;
    // std::vector<cv::KeyPoint> pre_frame_keypoints_;
    // std::vector<cv::KeyPoint> curr_frame_keypoints_;
    std::vector<cv::Point2f> pre_frame_keypoints_;
    std::vector<cv::Point2f> curr_frame_keypoints_;
    std::vector<unsigned long long int> keypoints_id_;
    std::map<unsigned long long int, FeatureId> keypoints_featureid_;

    std::map<FeatureId, Feature> map_feature_set_;
    std::map<FeatureId, Feature> map_observation_set_;
    std::map<StateId, CameraState> map_state_set_;

    std::vector<int> inlier_markers;

private:
    bool is_first_ = true;
    double tracking_rate_ = 1.0;
    cv::Mat camera_mat_;
    cv::Mat dist_coeffs_;
    KalmanFilter::Ptr filter_;
    utiltool::ConfigInfo::Ptr config_;
    std::map<int, FeatureId> trainidx_feature_map_;
    utiltool::NavTime curr_time_;
    utiltool::NavInfo nav_info_;

    cv::Mat pre_img;

private:
    std::ofstream ofs_camera_state_log_file_;
    std::ofstream ofs_msckf_info_log_file_;
    std::ofstream ofs_debug_info_log_file_;
    std::ofstream ofs_feature_log_file_;
    // std::ofstream ofs_feature_log_file_;
    bool camera_state_log_enable_ = false;
    bool msckf_info_log_enable_ = false;
    bool debug_info_log_enable_ = false;
    bool feature_log_enable_ = false;
};

struct ReprojectionError
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //  // ReprojectionError(const cv::Point2f &measure_point_uv,
    //  //                   const Eigen::Isometry3d &cam0_cami_transform) : image_uv(measure_point_uv),
    //  //                                                                   cam0_cami(cam0_cami_transform) {}
    //  // template <typename T>
    //  // bool operator()(const T *const feature_point_world, T *residuals) const
    //  // {
    //  //     Eigen::Matrix<T, 3, 1> point;
    //  //     point << feature_point_world[0], feature_point_world[1], feature_point_world[2];//

    //  //     point = cam0_cami.inverse() * point;
    //  //     T point_reproject_image[2] = {point(0) / point(2), point(1) / point(2)};
    //  //     residuals[0] = point_reproject_image[0] - T(image_uv.x);
    //  //     residuals[1] = point_reproject_image[1] - T(image_uv.y);
    //  //     return true;
    //  // }

    ReprojectionError(const cv::Point2f &measure_point_uv,
                      const Eigen::Isometry3d &cam0_cami_transform) : image_uv(measure_point_uv)
    {
        Eigen::Isometry3d cami_cam0 = cam0_cami_transform.inverse();
        quat = utiltool::attitude::RotationMartix2Quaternion(cami_cam0.rotation());
        cam0_cami_translation[0] = cami_cam0.translation()(0);
        cam0_cami_translation[1] = cami_cam0.translation()(1);
        cam0_cami_translation[2] = cami_cam0.translation()(2);
    }
    template <typename T>
    bool operator()(const T *const feature_point_world, T *residuals) const
    {
        T point[3] = {T(feature_point_world[0]), T(feature_point_world[1]), T(1.0)};
        T point_result[3];
        T cam0_cami_quat[4] = {T(quat.w()),
                               T(quat.x()),
                               T(quat.y()),
                               T(quat.z())};
        T point_translation[3] = {T(feature_point_world[2] * cam0_cami_translation[0]),
                                  T(feature_point_world[2] * cam0_cami_translation[1]),
                                  T(feature_point_world[2] * cam0_cami_translation[2])};
        ceres::QuaternionRotatePoint(cam0_cami_quat, point, point_result);
        point_result[0] += point_translation[0];
        point_result[1] += point_translation[1];
        point_result[2] += point_translation[2];

        T point_reproject_image[2] = {T(point_result[0] / point_result[2]),
                                      T(point_result[1] / point_result[2])};
        residuals[0] = -point_reproject_image[0] + T(image_uv.x);
        residuals[1] = -point_reproject_image[1] + T(image_uv.y);
        return true;
    }

    static ceres::CostFunction *Create(const cv::Point2f &measure_point_uv,
                                       const Eigen::Isometry3d &cam0_cami_transform)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(new ReprojectionError(measure_point_uv, cam0_cami_transform));
    }

    const cv::Point2f image_uv;
    Eigen::Quaterniond quat;
    double cam0_cami_translation[3];
    // const Eigen::Isometry3d cam0_cami;
};
} // namespace camera
} // namespace mscnav

#endif /* !MSCKFPROCESS_H_ */
