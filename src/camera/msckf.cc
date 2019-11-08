/*
** msckf.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午8:36:30 2019 little fang
** Last update Sat Nov 8 下午8:46:31 2019 little fang
*/

#include "navattitude.hpp"
#include "camera/msckf.hpp"
#include "camera/imageprocess.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace utiltool::attitude;

namespace mscnav
{
namespace camera
{

MsckfProcess::MsckfProcess(const KalmanFilter::Ptr &filter) : filter_{filter}
{
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

    dist_coeffs_ = (cv::Mat_<double>(5, 1) << dist_par[0], dist_par[1], dist_par[2], dist_par[3], dist_par[4]);

    //* 外参设置
    auto cam_imu_rotation = config_->get_array<double>("camera_imu_rotation");
    auto cam_imu_translation = config_->get_array<double>("camera_imu_translation");
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    rotation << cam_imu_rotation[0], cam_imu_rotation[1], cam_imu_rotation[2],
        cam_imu_rotation[3], cam_imu_rotation[4], cam_imu_rotation[5],
        cam_imu_rotation[6], cam_imu_rotation[7], cam_imu_rotation[8];

    // TODO 需要核对,需要确定一般给定的cam和imu方向余弦矩阵由哪个旋转至哪个
    cam_imu_tranformation_ = {Eigen::Isometry3d::Identity()};
    translation << cam_imu_translation[0], cam_imu_translation[1], cam_imu_translation[2];
    cam_imu_tranformation_.rotate(rotation);
    cam_imu_tranformation_.pretranslate(translation);
    // cam_imu_tranformation_.translate(translation);
}

void MsckfProcess::FirstImageProcess(const cv::Mat &img1, const utiltool::NavInfo &navifo)
{
    ImageProcess::OrbFreatureExtract(img1, pre_frame_keypoints_, pre_frame_descriptors_);
    AguementedState(navifo);
    return;
}

bool MsckfProcess::ProcessImage(const cv::Mat &img1, const utiltool::NavTime &time, utiltool::NavInfo &navinfo)
{
    static int max_camera_size = config_->get<int>("max_camera_sliding_window");
    curr_time_ = time;
    if (is_first_)
    {
        pre_img = img1.clone();
        FirstImageProcess(img1, navinfo);
        is_first_ = false;
        return false;
    }

    //** 特征点提取
    ImageProcess::OrbFreatureExtract(img1,
                                     curr_frame_keypoints_,
                                     curr_frame_descriptors_);
    cv::Mat key_image;
    cv::drawKeypoints(img1, curr_frame_keypoints_, key_image, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("key points", key_image);
    cv::waitKey(1);

    //** 与前一帧图像匹配特征点
    ImageProcess::FreatureMatch(pre_frame_keypoints_,
                                pre_frame_descriptors_,
                                curr_frame_keypoints_,
                                curr_frame_descriptors_,
                                matches_, 20.0);
    // cv::drawMatches(pre_img, pre_frame_keypoints_, img1, curr_frame_keypoints_, matches_, key_image);
    // cv::imshow("匹配数据", key_image);
    // cv::waitKey(0);
    // pre_img = img1.clone();

    //** 增加当前camera State到系统中去.
    AguementedState(navinfo);

    //** 匹配的特征加入观测序列中
    AddObservation();

    //** 选择本次参与量测更新的点集
    DetermineMeasureFeature();

    //** 特征点量测更新
    FeatureMeasureUpdate(navinfo);

    //** 清除已经完成全部feature点量测的camera state
    RemoveCameraState();

    if (map_state_set_.size() > max_camera_size)
    {
        RemoveRedundantCamStates(navinfo);
    }
    return true;
}

/**
 * @brief  状态增广
 *         1.增加相机的状态到map_state_set_中去
 *         2.增广对应相机状态的方差协方差到滤波方差中去
 *         参考msckf:https://github.com/KumarRobotics/msckf_vio
 * @note   
 * @retval None
 */
void MsckfProcess::AguementedState(const utiltool::NavInfo &navinfo)
{
    //* 计算camera 的state状态，增加到CameraState map中去
    CameraState cam_state;
    //TODO 需要验证
    cam_state.position_ = navinfo.pos_ + navinfo.rotation_ * cam_imu_tranformation_.translation();
    cam_state.quat_ = (navinfo.rotation_ * cam_imu_tranformation_.rotation());
    cam_state.time_ = navinfo.time_;
    map_state_set_.insert(std::make_pair(cam_state.state_id_, cam_state));
    // map_state_set_[cam_state.state_id_] = cam_state;//!不要采用这种方式赋值，id号会不对

    //* 增广对应状态的方差信息到滤波的方程协方差矩阵中
    utiltool::StateIndex &index = filter_->GetStateIndex();
    int state_count = index.total_state + index.camera_state_index.size() * 6;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(state_count + 6, state_count);
    J.block(0, 0, state_count, state_count) = Eigen::MatrixXd::Identity(state_count, state_count);

    //TODO 需要核对是rotation还是其装置 三行参数都需要再核对
    //TODO 协方差传递需要重新推到，对应顺序不对 初步调正,需要核实！！！
    // !-> 需要核实！！！
    J.block<3, 3>(state_count, 6) = cam_imu_tranformation_.rotation();
    J.block<3, 3>(state_count + 3, 6) = navinfo.rotation_ * utiltool::skew(cam_imu_tranformation_.translation());
    J.block<3, 3>(state_count + 3, 0) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd &state_cov = filter_->GetStateCov();
    state_cov = J * state_cov * J.transpose();

    int the_latest_count_state = (index.total_state) + index.camera_state_index.size() * 6;
    index.camera_state_index[cam_state.state_id_] = the_latest_count_state;
    return;
}

/**
 * @brief  增加匹配到的特征点到观测序列中去
 *         1.如果当前匹配的特征点在上一帧中存在直接加入观测值到其observation中            
 *         2.如果是全新的特征点，则构建新的特征点，添加前后两帧图像的观测数据，并增加到feature的序列中去
 * @note   
 *          2019.11.07 第一次调试TODO内容:第一个camera状态观测不对
 * @retval None
 */
void MsckfProcess::AddObservation()
{
    std::map<int, FeatureId> curr_trainidx_feature_map_;
    auto curr_camera_state = map_state_set_.end();
    curr_camera_state--;
    auto pre_camera_state = curr_camera_state;
    pre_camera_state--;

    int track_num = 0;
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
            keypoint_distorted.push_back(pre_frame_keypoints_[member.trainIdx].pt); //TODO 核实这两个trainIdx 2019.11.07
            keypoint_distorted.push_back(curr_frame_keypoints_[member.queryIdx].pt);
            // keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            feature.observation_uv_[pre_camera_state->first] = keypoint_undistorted[0];
            feature.observation_uv_[curr_camera_state->first] = keypoint_undistorted[1];
            map_feature_set_.insert(std::make_pair(feature.feature_id_, feature));
            // map_feature_set_[feature.feature_id_] = feature;
            curr_trainidx_feature_map_[member.queryIdx] = feature.feature_id_;
            curr_camera_state->second.feature_id_set_.push_back(feature.feature_id_);
            pre_camera_state->second.feature_id_set_.push_back(feature.feature_id_);
        }
        else // 已经存在的特征点，直接在观测序列中加入当前观测
        {
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            map_feature_set_[iter_feature->second].observation_uv_[curr_camera_state->first] = keypoint_undistorted[0];
            curr_trainidx_feature_map_[member.queryIdx] = iter_feature->second;
            curr_camera_state->second.feature_id_set_.push_back(iter_feature->second);
            track_num++;
        }
    }
    trainidx_feature_map_ = curr_trainidx_feature_map_;
    pre_frame_descriptors_ = curr_frame_descriptors_.clone();
    pre_frame_keypoints_ = curr_frame_keypoints_;
    tracking_rate_ = (double)(track_num) / (double)(curr_camera_state->second.feature_id_set_.size());
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
            iter_member = map_feature_set_.erase(iter_member);
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

/**
 * @brief  三角化求解近似位置，参考系为第一帧相片的相机坐标系
 * @note   
 * @param  &feature: 
 * @param  &cam0_trans: 
 * @param  position_cam0[3]: 输出
 * @retval 
 */
bool MsckfProcess::TriangulatePoint(const Feature &feature,
                                    const Eigen::Isometry3d &cam0_trans,
                                    double position_cam0[3])
{
    const StateId &latest_id = feature.observation_uv_.rbegin()->first;

    Eigen::Isometry3d cami_trans{Eigen::Isometry3d::Identity()};
    cami_trans.rotate(map_state_set_[latest_id].quat_.toRotationMatrix());
    cami_trans.pretranslate(map_state_set_[latest_id].position_);
    Eigen::Isometry3d cam0_cami = (cami_trans.inverse() * cam0_trans).inverse();

    const Eigen::Matrix4d mat_tmp = cam0_cami.matrix();
    cv::Mat T2 = (cv::Mat_<double>(3, 4) << mat_tmp(0, 0), mat_tmp(0, 1), mat_tmp(0, 2), mat_tmp(0, 3),
                  mat_tmp(1, 0), mat_tmp(1, 1), mat_tmp(1, 2), mat_tmp(1, 3),
                  mat_tmp(2, 0), mat_tmp(2, 1), mat_tmp(2, 2), mat_tmp(2, 3));

    cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
                  0, 1, 0, 0, 0, 0, 1, 0);
    cv::Mat pts_4d;
    std::vector<cv::Point2f> pts1, pts2;
    pts1.emplace_back(feature.observation_uv_.begin()->second);
    pts2.emplace_back(feature.observation_uv_.rbegin()->second);
    cv::triangulatePoints(T1, T2, pts1, pts2, pts_4d);
    cv::Mat x = pts_4d.col(0);
    x /= x.at<float>(3, 0);
    position_cam0[0] = x.at<float>(0, 0);
    position_cam0[1] = x.at<float>(1, 0);
    position_cam0[2] = x.at<float>(2, 0);
    return true;
}

/**
 * @brief  LM优化求解特征点在世界坐标系下的位置
 * @note   
 * @param  &feature: 待求的特征点
 * @retval 
 */
bool MsckfProcess::LMOptimizatePosition(Feature &feature)
{
    //TODO ceres初步撰写，需要测试
    if (feature.is_initialized_)
        return true;
    ceres::Problem problem;
    Eigen::Isometry3d cam0_tranformation{Eigen::Isometry3d::Identity()};
    cam0_tranformation.rotate(map_state_set_[feature.observation_uv_.begin()->first].quat_.toRotationMatrix());
    cam0_tranformation.pretranslate(map_state_set_[feature.observation_uv_.begin()->first].position_);
    double position_world[3] = {1.0, 1.0, 1.0};
    TriangulatePoint(feature, cam0_tranformation, position_world);
    if (position_world[2] < -1.0 || position_world[2] > 50.0)
    {
        return false;
    }
    for (auto &observation : feature.observation_uv_)
    {
        auto &camera_id = observation.first;
        const auto &camera_state = map_state_set_[camera_id];//! FIXME 有问题
        cv::Point2f &observation_point = observation.second;
        Eigen::Isometry3d cami_tranformation{Eigen::Isometry3d::Identity()};
        cami_tranformation.rotate(camera_state.quat_.toRotationMatrix());
        cami_tranformation.pretranslate(camera_state.position_);
        Eigen::Isometry3d cam0_cami = (cami_tranformation.inverse() * cam0_tranformation).inverse();
        ceres::CostFunction *cost_function = ReprojectionError::Create(observation_point, cam0_cami);
        problem.AddResidualBlock(cost_function, NULL, position_world);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);

    feature.position_world_ << position_world[0], position_world[1], position_world[2];
    if (feature.position_world_.norm() > 50)
        return false;
    feature.position_world_ = cam0_tranformation * feature.position_world_;
    return true;
}

/**
 * @brief  对已经获得三维坐标的feature点进行量测更新
 * @note   
 * @param  &feature: 待更新的点
 * @param  &H_state: 对应的量测更新矩阵
 * @param  &z_measure: 该点对应的残差向量
 * @retval 
 */
bool MsckfProcess::MeasurementJacobian(const Feature &feature,
                                       Eigen::MatrixXd &H_state,
                                       Eigen::VectorXd &z_measure)
{
    int state_count = map_state_set_.size();
    int observe_feature_count = feature.observation_uv_.size();
    const utiltool::StateIndex &state_index = filter_->GetStateIndex();
    //* 一个相机状态观测到一个feature提供两个残差[u,v]
    H_state = Eigen::MatrixXd::Zero(observe_feature_count * 2, state_count * 6 + state_index.total_state);
    z_measure = Eigen::VectorXd::Zero(observe_feature_count * 2);
    Eigen::MatrixXd H_feature = Eigen::MatrixXd::Zero(observe_feature_count * 2, 3);

    int row_index = 0;
    for (auto member : feature.observation_uv_)
    {
        auto &camera_state = map_state_set_[member.first];
        auto &measure = member.second;
        auto &point_coor_world = feature.position_world_;
        auto iter_camera_index = state_index.camera_state_index.find(camera_state.state_id_);
        if (iter_camera_index == state_index.camera_state_index.end())
        {
            LOG(ERROR) << "find camera state index failed" << std::endl;
            continue;
        }
        Eigen::Vector3d l_cf_w = (point_coor_world - camera_state.position_);
        Eigen::Vector3d point_f_c = camera_state.quat_.conjugate() * l_cf_w;
        Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(2, 3);
        Eigen::MatrixXd J2_1 = Eigen::MatrixXd::Zero(3, 6);
        Eigen::MatrixXd J2_2 = Eigen::MatrixXd::Zero(3, 3);
        double z_1 = 1.0 / point_coor_world(2);
        J1 << z_1, 0, -point_coor_world(0) * z_1 * z_1,
            0, z_1, -point_coor_world(1) * z_1 * z_1;
        J2_1.block<3, 3>(0, 0) = camera_state.quat_.toRotationMatrix().transpose() *
                                 utiltool::skew(l_cf_w);
        J2_1.block<3, 3>(0, 3) = -1 * camera_state.quat_.toRotationMatrix().transpose();
        J2_2 = camera_state.quat_.toRotationMatrix().transpose();
        H_feature.block<2, 3>(row_index, 0) = J1 * J2_2;
        H_state.block<2, 6>(row_index, iter_camera_index->second) = J1 * J2_1;
        z_measure.segment<2>(row_index) << measure.x - point_f_c(0) / point_f_c(2),
            measure.y - point_f_c(1) / point_f_c(2);
        row_index += 2;
    }

    //* SVD求解Hf的左零空间，消除Hf
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(H_feature, Eigen::ComputeFullU | Eigen::ComputeThinV);
    Eigen::MatrixXd A = svd_helper.matrixU().rightCols(observe_feature_count * 2 - 3);
    H_state = A.transpose() * H_state;
    z_measure = A.transpose() * z_measure;
    return true;
}

/**
 * @brief  确定量测噪声，进行量测更新，返回dx
 * @note   
 * @param  &H_state: 量测矩阵
 * @param  &z_measure: 残差矩阵
 * @retval 修正量
 */
Eigen::VectorXd MsckfProcess::MeasurementUpdate(const Eigen::MatrixXd &H_state,
                                                const Eigen::VectorXd &z_measure)
{
    static double pixel_noise = config_->get<double>("camera_pixel_noise");
    Eigen::MatrixXd measure_noise = Eigen::MatrixXd::Identity(H_state.rows(), H_state.rows());
    measure_noise = measure_noise * (pixel_noise * pixel_noise);
    return filter_->MeasureUpdate(H_state, z_measure, measure_noise, curr_time_);
}

/**
 * @brief  对相机状态进行反馈，更新相机的位姿
 * @note   
 * @param  &dx_camera: 
 * @retval None
 */
void MsckfProcess::ReviseCameraState(const Eigen::VectorXd &dx_camera)
{
    int camera_state_count = 0;
    // map_state_set_.size();
    for (auto &camera_state : map_state_set_)
    {
        Eigen::Vector3d theta_camera = dx_camera.segment<3>(camera_state_count * 6);
        Eigen::Vector3d t_camera = dx_camera.segment<3>(camera_state_count * 6 + 3);
        camera_state_count++;
        auto camera_quat_delta = RotationVector2Quaternion(theta_camera);
        camera_state.second.quat_ = camera_quat_delta * camera_state.second.quat_;
        camera_state.second.position_ += t_camera;
    }
    return;
}

/**
 * @brief  移除多余的camera状态量，对feature完全没有观测了
 * @note   
 * @retval None
 */
void MsckfProcess::RemoveCameraState()
{
    auto &index = filter_->GetStateIndex();
    for (auto iter_camera = map_state_set_.begin(); iter_camera != map_state_set_.end();)
    {
        bool find_feature = false;
        auto &camera_state = iter_camera->second;
        for (auto f_id : camera_state.feature_id_set_)
        {
            if (map_feature_set_.find(f_id) != map_feature_set_.end())
            {
                find_feature = true;
                break;
            }
        }
        if (find_feature)
        {
            iter_camera++;
            continue;
        }
        else
        {
            auto iter_index_camera = index.camera_state_index.find(iter_camera->first);
            assert(iter_index_camera != index.camera_state_index.end());
            filter_->EliminateIndex(iter_index_camera->second, 6);
            iter_index_camera = index.camera_state_index.erase(iter_index_camera);
            for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
            {
                iter_index_camera->second -= 6;
            }
            iter_camera = map_state_set_.erase(iter_camera);
        }
    }
}

/**
 * @brief  循环更新全部的量测feature点集
 * @note   
 * @retval None
 */
void MsckfProcess::FeatureMeasureUpdate(utiltool::NavInfo &navinfo)
{
    for (auto iter_feature = map_observation_set_.begin(); iter_feature != map_observation_set_.end(); iter_feature++)
    //for (auto iter_feature = map_feature_set_.begin(); iter_feature != map_feature_set_.end(); iter_feature++)
    {
        bool temp = LMOptimizatePosition(iter_feature->second);
        if (temp)
        {
            Eigen::MatrixXd H_state;
            Eigen::VectorXd z_measure;
            iter_feature->second.is_initialized_ = true;

            MeasurementJacobian(iter_feature->second, H_state, z_measure);
            Eigen::VectorXd dx = MeasurementUpdate(H_state, z_measure);
            filter_->ReviseState(navinfo, dx.head(filter_->GetStateIndex().total_state));
            ReviseCameraState(dx.tail(dx.size() - filter_->GetStateIndex().total_state));
        }
    }
    map_observation_set_.clear();
}

/**
 * @brief  查找移除的两个状态量,并更新对应的特征点
 * @note   
 * @retval None
 */
void MsckfProcess::RemoveRedundantCamStates(utiltool::NavInfo &navinfo)
{
    std::vector<StateId> rm_cam_state_ids;
    auto key_cam_state_iter = map_state_set_.end();
    for (int i = 0; i < 4; ++i)
        --key_cam_state_iter;
    auto cam_state_iter = key_cam_state_iter;
    ++cam_state_iter;
    auto first_cam_state_iter = map_state_set_.begin();

    const Eigen::Vector3d key_position = key_cam_state_iter->second.position_;
    const Eigen::Matrix3d key_rotation = key_cam_state_iter->second.quat_.toRotationMatrix();

    for (int i = 0; i < 2; ++i)
    {
        const Eigen::Vector3d position = cam_state_iter->second.position_;
        const Eigen::Matrix3d rotation = cam_state_iter->second.quat_.toRotationMatrix();

        double distance = (position - key_position).norm();
        double angle = Eigen::AngleAxisd(rotation * key_rotation.transpose()).angle();

        if (angle < 0.2618 && distance < 0.4 && tracking_rate_ > 0.5)
        {
            rm_cam_state_ids.push_back(cam_state_iter->first);
            ++cam_state_iter;
        }
        else
        {
            rm_cam_state_ids.push_back(first_cam_state_iter->first);
            ++first_cam_state_iter;
        }
    }
    std::sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

    //TODO 更新这两个相机状态可以观测到feature点
    map_observation_set_.clear();
    for (auto &member : map_feature_set_)
    {
        auto &feature = member.second;
        auto iter_cam1 = feature.observation_uv_.find(rm_cam_state_ids[0]);
        auto iter_cam2 = feature.observation_uv_.find(rm_cam_state_ids[1]);
        if (iter_cam1 != feature.observation_uv_.end() && iter_cam2 != feature.observation_uv_.end())
        {
            if (LMOptimizatePosition(feature))
            {
                Feature tmp_feature(feature.feature_id_);
                tmp_feature.observation_uv_[rm_cam_state_ids[0]] = iter_cam1->second;
                tmp_feature.observation_uv_[rm_cam_state_ids[1]] = iter_cam2->second;
                tmp_feature.is_initialized_ = true;
                tmp_feature.position_world_ = feature.position_world_;
                feature.is_initialized_ = false;
                map_observation_set_[tmp_feature.feature_id_] = tmp_feature;
            }
            feature.observation_uv_.erase(iter_cam1);
            feature.observation_uv_.erase(iter_cam2);
            continue;
        }
        if (iter_cam1 != feature.observation_uv_.end())
        {
            feature.observation_uv_.erase(iter_cam1);
        }
        if (iter_cam2 != feature.observation_uv_.end())
        {
            feature.observation_uv_.erase(iter_cam2);
        }
    }

    //* 更新状态
    FeatureMeasureUpdate(navinfo);

    //** 移除这两个相机状态
    auto &index = filter_->GetStateIndex();
    for (auto iter_camera : rm_cam_state_ids)
    {
        /* code */
        auto iter_index_camera = index.camera_state_index.find(iter_camera);
        assert(iter_index_camera != index.camera_state_index.end());
        filter_->EliminateIndex(iter_index_camera->second, 6);
        iter_index_camera = index.camera_state_index.erase(iter_index_camera);
        for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
        {
            iter_index_camera->second -= 6;
        }
        map_state_set_.erase(iter_camera);
    }
}

} // namespace camera

} // namespace mscnav