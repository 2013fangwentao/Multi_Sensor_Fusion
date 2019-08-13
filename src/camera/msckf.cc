/*
** msckf.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午8:36:30 2019 little fang
** Last update Mon Aug 11 下午2:52:23 2019 little fang
*/

#include "camera/msckf.hpp"
#include "camera/imageprocess.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace mscnav
{
namespace camera
{

MsckfProcess::MsckfProcess(const KalmanFilter::Ptr &filter) : filter_{filter}
{
    state_ = State::GetState();
    config_ = utiltool::ConfigInfo::GetInstance();
    int num_features = config_->get<int>("number_points_per_image");
    int num_levels = config_->get<int>("number_pyramid_levels");
    float scale_factor = config_->get<float>("scale_factor");
    int ini_th_fast = config_->get<int>("iniThFAST");
    int min_th_fast = config_->get<int>("minThFAST");
    ImageProcess::Initialize(num_features, scale_factor, num_levels, ini_th_fast, min_th_fast);
    
    //* 相机内参和畸变矫正参数
    auto camera_par = config_->get_array<double>("camera_intrinsic");
    auto dist_par = config_->get_array<double>("camera_distcoeffs");
    if (camera_par.size() != 4 || dist_par.size() != 5)
    {
        LOG(ERROR) << "camera_intrinsic/camera_distcoeffs size error" << std::endl;
        getchar();
    }
    camera_mat_ = (cv::Mat_<double>(3, 3) << camera_par[0], 0.0, camera_par[2],
                   0.0, camera_par[1], camera_par[3],
                   0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat_<double>(5, 1) << dist_par[0],dist_par[1],dist_par[2],dist_par[3],dist_par[4]);

    //* 外参设置
    auto cam_imu_rotation = config_->get_array<double>("camera_imu_rotation");
    auto cam_imu_translation = config_->get_array<double>("camera_imu_translation");
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    rotation << cam_imu_rotation[0], cam_imu_rotation[1], cam_imu_rotation[2],
        cam_imu_rotation[3], cam_imu_rotation[4], cam_imu_rotation[5],
        cam_imu_rotation[6], cam_imu_rotation[7], cam_imu_rotation[8];

    // TODO 需要核对,需要确定一般给定的cam和imu方向余弦矩阵由哪个旋转至哪个
    translation << cam_imu_translation[0], cam_imu_translation[1], cam_imu_translation[2];
    cam_imu_tranformation_.rotate(rotation);
    cam_imu_tranformation_.translate(translation);
}

void MsckfProcess::FirstImageProcess(const cv::Mat &img1)
{
    ImageProcess::OrbFreatureExtract(img1, pre_frame_keypoints_, pre_frame_descriptors_);
    AguementedState();
    return;
}

bool MsckfProcess::ProcessImage(const cv::Mat &img1, Eigen::VectorXd &dx)
{
    if (is_first_)
    {
        FirstImageProcess(img1);
        is_first_ = false;
        return false;
    }

    //** 增加当前camera State到系统中去.
    AguementedState();

    //** 特征点提取
    ImageProcess::OrbFreatureExtract(img1,
                                     curr_frame_keypoints_,
                                     curr_frame_descriptors_);

    //** 与前一帧图像匹配特征点
    ImageProcess::FreatureMatch(curr_frame_keypoints_,
                                curr_frame_descriptors_,
                                pre_frame_keypoints_,
                                pre_frame_descriptors_,
                                matches_);
    //** 匹配的特征加入观测序列中
    AddObservation();

    //** 选择本次参与量测更新的点集
    DetermineMeasureFeature();
}

/**
 * @brief  状态增广
 *         1.增加相机的状态到map_state_set_中去
 *         2.增广对应相机状态的方差协方差到滤波方差中去
 *         参考msckf:https://github.com/KumarRobotics/msckf_vio
 * @note   
 * @retval None
 */
void MsckfProcess::AguementedState()
{
    //* 计算camera 的state状态，增加到CameraState map中去
    CameraState cam_state;
    auto navinfo = state_->GetNavInfo();
    //TODO 需要验证
    cam_state.position_ = navinfo.pos_ + navinfo.rotation_ * cam_imu_tranformation_.translation();
    cam_state.quat_ = (navinfo.rotation_ * cam_imu_tranformation_.rotation());
    cam_state.time_ = navinfo.time_;
    map_state_set_[cam_state.state_id_] = cam_state;

    //* 增广对应状态的方差信息到滤波的方程协方差矩阵中
    utiltool::StateIndex &index = filter_->GetStateIndex();
    int state_count = index.total_state + index.camera_state_index.size();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(state_count + 6, state_count);
    J.block(0, 0, state_count, state_count) = Eigen::MatrixXd::Identity(state_count, state_count);

    //TODO 需要核对是rotation还是其装置 三行参数都需要再核对
    J.block<3, 3>(state_count, 0) = cam_imu_tranformation_.rotation();
    J.block<3, 3>(state_count + 3, 0) = navinfo.rotation_ * utiltool::skew(cam_imu_tranformation_.translation());
    J.block<3, 3>(state_count + 3, 6) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd &state_cov = filter_->GetStateCov();
    state_cov = J * state_cov * J.transpose();

    int the_latest_count_state = (index.total_state) + index.camera_state_index.size() * 6;
    index.camera_state_index[cam_state.state_id_] = the_latest_count_state;
}

/**
 * @brief  增加匹配到的特征点到观测序列中去
 *         1.如果当前匹配的特征点在上一帧中存在直接加入观测值到其observation中            
 *         2.如果是全新的特征点，则构建新的特征点，添加前后两帧图像的观测数据，并增加到feature的序列中去
 * @note   
 * @retval None
 */
void MsckfProcess::AddObservation()
{
    std::map<int, FeatureId> curr_trainidx_feature_map_;
    auto curr_camera_state = map_state_set_.end();
    curr_camera_state--;
    auto pre_camera_state = curr_camera_state;
    pre_camera_state--;

    //TODO 此处在后续测试中需要重点关注，可能存在很多bug
    std::vector<cv::Point2f> keypoint_distorted, keypoint_undistorted;
    for (auto &member : matches_)
    {
        auto iter_feature = trainidx_feature_map_.find(member.trainIdx);
        if (iter_feature == trainidx_feature_map_.end()) // 新的特征点，创建实例对象，并赋值
        {
            Feature feature;
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.push_back(pre_frame_keypoints_[member.trainIdx].pt);
            keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            feature.observation_uv_[pre_camera_state->first] = keypoint_undistorted[0];
            feature.observation_uv_[curr_camera_state->first] = keypoint_undistorted[1];
            map_feature_set_[feature.feature_id_] = feature;
            curr_trainidx_feature_map_[member.queryIdx] = feature.feature_id_;
        }
        else // 已经存在的特征点，直接在观测序列中加入当前观测
        {
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            map_feature_set_[iter_feature->second].observation_uv_[curr_camera_state->first] = keypoint_undistorted[0];
            curr_trainidx_feature_map_[member.queryIdx] = iter_feature->second;
        }
    }
    trainidx_feature_map_ = curr_trainidx_feature_map_;
    pre_frame_descriptors_ = curr_frame_descriptors_;
    pre_frame_keypoints_ = curr_frame_keypoints_;
}

/**
 * @brief  决定当前时刻使用多少特征点进行量测更新
 * @note   1.不在当前帧中可以观测到的点备选为可以进行量测更新的点
 *         2.[1]中观测次数不超过三次的点，不参与量测更新
 *         3.[1]无法进行三角化的点剔除，不参与量测更新
 * @retval None
 */
void MsckfProcess::DetermineMeasureFeature()
{
    map_observation_set_.clear();
    for (auto iter_member = map_feature_set_.begin(); iter_member != map_feature_set_.end();)
    {
        auto iter_feature_inmap = std::find_if(trainidx_feature_map_.begin(), trainidx_feature_map_.end(), [&](std::pair<int, long long int> feature_iter) {
            return feature_iter.second == iter_member->first;
        });
        if (iter_feature_inmap == trainidx_feature_map_.end())
        {
            auto &feature = iter_member->second;
            if (feature.observation_uv_.size() > 3)
            {
                if (CheckEnableTriangleate(feature))
                {
                    map_observation_set_.insert(*iter_member);
                }
            }
            map_feature_set_.erase(iter_member);
            continue;
        }
        iter_member++;
    }
}

/**
 * @brief  判断对的特征点能否进行三角化
 * @note   
 * @param  &feature: 待判断的特征点
 * @retval -true  -false
 */
bool MsckfProcess::CheckEnableTriangleate(const Feature &feature)
{
    if (feature.is_initialized_)
        return true;

    auto iter_feature_observation = feature.observation_uv_.begin();
    auto &the_first_camera = map_state_set_[iter_feature_observation->first];

    //* 获取第一个相机坐标系中特征点的向量方向在世界坐标系的投影
    Eigen::Vector3d feature_direction{
        iter_feature_observation->second.x,
        iter_feature_observation->second.y,
        1.0};
    feature_direction = feature_direction / feature_direction.norm();
    feature_direction = the_first_camera.quat_.toRotationMatrix() * feature_direction;

    //* 参考msckf-vio
    //* https://github.com/KumarRobotics/msckf_vio
    iter_feature_observation = feature.observation_uv_.end();
    iter_feature_observation--;
    auto &the_last_camera = map_state_set_[iter_feature_observation->first];
    Eigen::Vector3d translation = the_last_camera.position_ - the_first_camera.position_;
    double parallel_translation = translation.transpose() * feature_direction;
    Eigen::Vector3d orthogonal_translation = translation - parallel_translation * feature_direction;
    if (orthogonal_translation.norm() > 0.2) //TODO 三角化指标 初步设置为0.2,后续需要测试调整
        return true;
    else
        return false;
}

bool MsckfProcess::LMOptimizatePosition(Feature &feature)
{

    // cv::triangulatePoints()
}
} // namespace camera

} // namespace mscnav