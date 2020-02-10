/*
** msckf.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午8:36:30 2019 little fang
** Last update Tue Feb 10 下午3:58:08 2020 little fang
*/

#include "navattitude.hpp"
#include "camera/msckf.hpp"
#include "camera/imageprocess.h"
#include "navbase.hpp"
#include <navearth.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

using namespace utiltool::attitude;
using namespace utiltool;
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
    camera_feature_log_ = (config_->get<int>("camera_feature_log_enable") == 0) ? false : true;
    if (camera_feature_log_)
    {
        std::string result_path = config_->get<std::string>("result_output_path");
        utiltool::NavTime time = utiltool::NavTime::NowTime();
        result_path.append("/camera_fearture_log.log-" + time.Time2String("%04d-%02d-%02d-%02d-%02d-%.1f"));
        ofs_camera_feature_log_file_.open(result_path);
        if (!ofs_camera_feature_log_file_.good())
        {
            LOG(ERROR) << "Camera and Feature log file open failed. Path: " << result_path << std::endl;
            getchar();
        }
    }
    ImageProcess::Initialize(num_features, scale_factor, num_levels, ini_th_fast, min_th_fast);

    //* 相机内参和畸变矫正参数
    auto camera_par = config_->get_array<double>("camera_intrinsic");
    auto dist_par = config_->get_array<double>("camera_distcoeffs");
    if (camera_par.size() != 4 || dist_par.size() != 5)
    {
        LOG(ERROR) << "camera_intrinsic/camera_distcoeffs size error" << std::endl;
        getchar();
    }
    camera_mat_ = (cv::Mat_<double>(3, 3) << camera_par.at(0), 0.0, camera_par.at(2),
                   0.0, camera_par.at(1), camera_par.at(3),
                   0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat_<double>(5, 1) << dist_par[0], dist_par[1], dist_par[2], dist_par[3], dist_par[4]);

    //* 外参设置
    auto cam_imu_rotation = config_->get_array<double>("camera_imu_rotation");
    auto cam_imu_translation = config_->get_array<double>("camera_imu_translation");
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    rotation << cam_imu_rotation.at(0), cam_imu_rotation.at(1), cam_imu_rotation.at(2),
        cam_imu_rotation.at(3), cam_imu_rotation.at(4), cam_imu_rotation.at(5),
        cam_imu_rotation.at(6), cam_imu_rotation.at(7), cam_imu_rotation.at(8);

    cam_imu_tranformation_ = {Eigen::Isometry3d::Identity()};
    translation << cam_imu_translation.at(0), cam_imu_translation.at(1), cam_imu_translation.at(2);
    cam_imu_tranformation_.rotate(rotation);
    cam_imu_tranformation_.pretranslate(translation);
    LOG(INFO) << "cam_imu: \n"
              << rotation << std::endl;
    LOG(INFO) << translation.transpose() << std::endl;
    // cam_imu_tranformation_.translate(translation);
}

// void MsckfProcess::FirstImageProcess(const cv::Mat &img1, const utiltool::NavInfo &navifo)
// {
//     ImageProcess::OrbFreatureExtract(img1, pre_frame_keypoints_, pre_frame_descriptors_);
//     AguementedState(navifo);
//     return;
// }

void MsckfProcess::FirstImageProcess(const cv::Mat &img1, const utiltool::NavInfo &navifo)
{
    ImageProcess::GoodFreatureDetect(img1, keypoints_id_, pre_frame_keypoints_);
    AguementedState(navifo);
    return;
}

bool MsckfProcess::ProcessImage(const cv::Mat &img_raw, const utiltool::NavTime &time, utiltool::NavInfo &navinfo)
{
    cv::Mat img1 = img_raw.clone();
    // cv::undistort(img_raw, img1, camera_mat_, dist_coeffs_);

    static int max_camera_size = config_->get<int>("max_camera_sliding_window");
    curr_time_ = time;
    if (is_first_)
    {
        FirstImageProcess(img1, navinfo);
        cv::swap(pre_img, img1);
        is_first_ = false;
        return false;
    }

    //** 增加当前camera State到系统中去.
    AguementedState(navinfo);

    // 光流追踪
    ImageProcess::LKTrack(pre_img, img1, keypoints_id_, pre_frame_keypoints_, curr_frame_keypoints_);

    // 外点剔除
    // LOG(INFO) << map_state_set_.size() << std::endl;
    auto curr_camera_state = map_state_set_.rbegin();
    auto pre_camera_state = curr_camera_state;
    pre_camera_state++;
    Eigen::Matrix3d rotation_imu = attitude::Quaternion2RotationMatrix(
        curr_camera_state->second.quat_.conjugate() * pre_camera_state->second.quat_);
    cv::Mat R_c;
    cv::eigen2cv(rotation_imu, R_c);

    ImageProcess::TwoPointRansac(pre_frame_keypoints_,
                                 curr_frame_keypoints_,
                                 R_c,
                                 camera_mat_,
                                 dist_coeffs_,
                                 1,
                                 0.999,
                                 inlier_markers);
    assert(inlier_markers.size() == pre_frame_keypoints_.size());

    cv::Mat key_image = img1.clone();
    for (int i = 0; i < curr_frame_keypoints_.size(); i++)
    {
        if (inlier_markers[i] == 0)
            continue;
        circle(key_image, curr_frame_keypoints_[i], 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    cv::imshow("key_image", key_image);
    cv::waitKey(1);

    // Test();

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

    ImageProcess::GoodFreatureDetect(img1, keypoints_id_, curr_frame_keypoints_);

    std::swap(pre_frame_keypoints_, curr_frame_keypoints_);
    cv::swap(pre_img, img1);
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

    cam_state.position_ = navinfo.pos_ + navinfo.rotation_ * cam_imu_tranformation_.translation();
    cam_state.quat_ = attitude::RotationMartix2Quaternion(navinfo.rotation_ * cam_imu_tranformation_.rotation());
    cam_state.time_ = navinfo.time_;
    map_state_set_.insert(std::make_pair(cam_state.state_id_, cam_state));
    // map_state_set_[cam_state.state_id_] = cam_state;//!不要采用这种方式赋值，id号会不对

    //* debug code DEBUG
    // auto BLH = earth::WGS84XYZ2BLH(cam_state.position_);
    // auto att = attitude::RotationMartix2Euler(earth::CalCe2n(BLH(0), BLH(1)) * cam_state.quat_.toRotationMatrix());
    //
    // LOG(INFO) << "camera rotation:" << att.transpose() * constant::rad2deg << std::endl
    //           << "imu rotation:" << (navinfo.att_).transpose() * constant::rad2deg << std::endl;

    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << cam_state.time_ << cam_state << std::endl;
    //     ofs_camera_feature_log_file_ << "map camera: " << map_state_set_ << std::endl;
    // }

    //* 增广对应状态的方差信息到滤波的方程协方差矩阵中
    utiltool::StateIndex &index = filter_->GetStateIndex();
    int state_count = index.total_state + index.camera_state_index.size() * 6;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(state_count + 6, state_count);
    J.block(0, 0, state_count, state_count) = Eigen::MatrixXd::Identity(state_count, state_count);

    // !-> 需要核实！！！2020-02-02核实错误，对应顺序不对，重新调整！特别要注意顺序！
    // J.block<3, 3>(state_count, 6) = cam_imu_tranformation_.rotation();
    // J.block<3, 3>(state_count + 3, 6) = navinfo.rotation_ * utiltool::skew(cam_imu_tranformation_.translation());
    // J.block<3, 3>(state_count + 3, 0) = Eigen::Matrix3d::Identity();

    J.block<3, 3>(state_count + 3, index.att_index_) = Eigen::Matrix3d::Identity(); //cam_imu_tranformation_.rotation().transpose();
    J.block<3, 3>(state_count, index.att_index_) = utiltool::skew(navinfo.rotation_ * cam_imu_tranformation_.translation());
    J.block<3, 3>(state_count, index.pos_index_) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd &state_cov = filter_->GetStateCov();
    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << std::fixed << std::setprecision(8) << "J:" << std::endl
    //                                  << J << std::endl;
    //     ofs_camera_feature_log_file_ << "state_cov:" << std::endl
    //                                  << state_cov << std::endl;
    // }
    state_cov = J * state_cov * J.transpose();
    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "state_cov2:" << std::endl
    //                                  << filter_->GetStateCov() << std::endl;
    // }

    int the_latest_count_state = (index.total_state) + index.camera_state_index.size() * 6;
    index.camera_state_index[cam_state.state_id_] = the_latest_count_state;
    if (camera_feature_log_)
    {
        ofs_camera_feature_log_file_ << "camera state index: " << index.camera_state_index << std::endl
                                     << std::endl;
    }
    return;
}

// /**
//  * @brief  增加匹配到的特征点到观测序列中去
//  *         1.如果当前匹配的特征点在上一帧中存在直接加入观测值到其observation中
//  *         2.如果是全新的特征点，则构建新的特征点，添加前后两帧图像的观测数据，并增加到feature的序列中去
//  * @note
//  *          2019.11.07 第一次调试TODO内容:第一个camera状态观测不对
//  * @retval None
//  */
// void MsckfProcess::AddObservation()
// {
//         std::map<int, FeatureId> curr_trainidx_feature_map_;
//         auto curr_camera_state = map_state_set_.end();
//         curr_camera_state--;
//         auto pre_camera_state = curr_camera_state;
//         pre_camera_state--;

//         int track_num = 0;

//         std::vector<cv::Point2f> keypoint_distorted, keypoint_undistorted;
//         for (auto &member : matches_)
//         {
//             auto iter_trainIdx = curr_trainidx_feature_map_.find(member.trainIdx);
//             if (iter_trainIdx != curr_trainidx_feature_map_.end())
//                 continue;
//             auto iter_feature = trainidx_feature_map_.find(member.queryIdx);
//             if (iter_feature == trainidx_feature_map_.end()) // 新的特征点，创建实例对象，并赋值
//             {
//                 Feature feature;
//                 keypoint_distorted.clear();
//                 keypoint_undistorted.clear();
//                 // keypoint_distorted.push_back(pre_frame_keypoints_[member.trainIdx].pt);
//                 keypoint_distorted.push_back(pre_frame_keypoints_[member.queryIdx].pt); //* 2020-02-01 核实trainIdx和queryIdx通过
//                 keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
//                 // cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
//                 NormKeyPoints(keypoint_distorted, keypoint_undistorted, camera_mat_);

//                 double diff_x = fabs(keypoint_undistorted[0].x - keypoint_undistorted[1].x);
//                 double diff_y = fabs(keypoint_undistorted[0].y - keypoint_undistorted[1].y);
//                 if (diff_y > 0.04 || diff_x > 0.12)
//                     continue;

//                 feature.observation_uv_[pre_camera_state->first] = keypoint_undistorted[0];
//                 feature.observation_uv_[curr_camera_state->first] = keypoint_undistorted[1];

//                 feature.raw_uv_[pre_camera_state->first] = keypoint_distorted[0];
//                 feature.raw_uv_[curr_camera_state->first] = keypoint_distorted[1];

//                 map_feature_set_.insert(std::make_pair(feature.feature_id_, feature));
//                 // map_feature_set_[feature.feature_id_] = feature;
//                 curr_trainidx_feature_map_[member.trainIdx] = feature.feature_id_;
//                 curr_camera_state->second.feature_id_set_.push_back(feature.feature_id_);
//                 pre_camera_state->second.feature_id_set_.push_back(feature.feature_id_);
//             }
//             else // 已经存在的特征点，直接在观测序列中加入当前观测
//             {
//                 keypoint_distorted.clear();
//                 keypoint_undistorted.clear();
//                 keypoint_distorted.push_back(curr_frame_keypoints_[member.trainIdx].pt);
//                 // cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
//                 NormKeyPoints(keypoint_distorted, keypoint_undistorted, camera_mat_);

//                 auto &pts = map_feature_set_[iter_feature->second].observation_uv_[pre_camera_state->first];
//                 double diff_x = fabs(keypoint_undistorted[0].x - pts.x);
//                 double diff_y = fabs(keypoint_undistorted[0].y - pts.y);
//                 if (diff_y > 0.04 || diff_x > 0.12)
//                     continue;

//                 map_feature_set_[iter_feature->second].observation_uv_[curr_camera_state->first] = keypoint_undistorted[0];
//                 map_feature_set_[iter_feature->second].raw_uv_[curr_camera_state->first] = keypoint_distorted[0];

//                 curr_trainidx_feature_map_[member.trainIdx] = iter_feature->second;
//                 curr_camera_state->second.feature_id_set_.push_back(iter_feature->second);
//                 track_num++;
//             }
//         }

//         if (camera_feature_log_)
//         {
//             ofs_camera_feature_log_file_ << "feature map:\t" << std::endl;
//             for (auto index : map_feature_set_)
//             {
//                 ofs_camera_feature_log_file_ << index.first << " " << index.second << std::endl;
//             }
//             ofs_camera_feature_log_file_ << std::endl;
//         }

//         trainidx_feature_map_ = curr_trainidx_feature_map_;
//         pre_frame_descriptors_ = curr_frame_descriptors_.clone();
//         pre_frame_keypoints_ = curr_frame_keypoints_;
//         tracking_rate_ = (double)(track_num) / (double)(curr_camera_state->second.feature_id_set_.size());
// }

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
    std::map<unsigned long long int, FeatureId> curr_trainidx_feature_map_;
    auto curr_camera_state = map_state_set_.end();
    curr_camera_state--;
    auto pre_camera_state = curr_camera_state;
    pre_camera_state--;

    int track_num = 0;
    int size = keypoints_id_.size();
    assert(size == curr_frame_keypoints_.size());
    assert(size == pre_frame_keypoints_.size());
    std::vector<cv::Point2f> keypoint_distorted, keypoint_undistorted;
    ofs_camera_feature_log_file_ << "outline:" << std::endl;
    for (size_t i = 0; i < size; i++)
    {
        ofs_camera_feature_log_file_ << inlier_markers.at(i) << " ";
        if ((i + 1) % 10 == 0)
            ofs_camera_feature_log_file_ << std::endl;
        if (inlier_markers.at(i) == 0)
            continue;
        auto iter_point_id = keypoints_featureid_.find(keypoints_id_.at(i));
        if (iter_point_id == keypoints_featureid_.end())
        {
            Feature feature;
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.emplace_back(pre_frame_keypoints_.at(i));
            keypoint_distorted.emplace_back(curr_frame_keypoints_.at(i));
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            // NormKeyPoints(keypoint_distorted, keypoint_undistorted, camera_mat_);

            // double diff_x = fabs(keypoint_undistorted[0].x - keypoint_undistorted[1].x);
            // double diff_y = fabs(keypoint_undistorted[0].y - keypoint_undistorted[1].y);
            // if (diff_y > 0.04 || diff_x > 0.12)
            //     continue;

            feature.observation_uv_[pre_camera_state->first] = keypoint_undistorted[0];
            feature.observation_uv_[curr_camera_state->first] = keypoint_undistorted[1];

            feature.raw_uv_[pre_camera_state->first] = keypoint_distorted[0];
            feature.raw_uv_[curr_camera_state->first] = keypoint_distorted[1];

            map_feature_set_.insert(std::make_pair(feature.feature_id_, feature));
            // map_feature_set_[feature.feature_id_] = feature;
            curr_trainidx_feature_map_[keypoints_id_.at(i)] = feature.feature_id_;
            curr_camera_state->second.feature_id_set_.emplace_back(feature.feature_id_);
            pre_camera_state->second.feature_id_set_.emplace_back(feature.feature_id_);
        }
        else
        {
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.emplace_back(curr_frame_keypoints_.at(i));
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);

            // NormKeyPoints(keypoint_distorted, keypoint_undistorted, camera_mat_);

            // auto &pts = map_feature_set_[iter_point_id->second].observation_uv_[pre_camera_state->first];
            // double diff_x = fabs(keypoint_undistorted[0].x - pts.x);
            // double diff_y = fabs(keypoint_undistorted[0].y - pts.y);
            // if (diff_y > 0.04 || diff_x > 0.12)
            //     continue;

            map_feature_set_[iter_point_id->second].observation_uv_[curr_camera_state->first] = keypoint_undistorted[0];
            map_feature_set_[iter_point_id->second].raw_uv_[curr_camera_state->first] = keypoint_distorted[0];

            curr_trainidx_feature_map_[keypoints_id_.at(i)] = iter_point_id->second;
            curr_camera_state->second.feature_id_set_.emplace_back(iter_point_id->second);
            track_num++;
        }
    }
    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "feature map:\t" << std::endl;
    //     for (auto index : map_feature_set_)
    //     {
    //         ofs_camera_feature_log_file_ << index.first << " " << index.second << std::endl;
    //     }
    //     ofs_camera_feature_log_file_ << std::endl;
    // }

    keypoints_featureid_ = curr_trainidx_feature_map_;
    // tracking_rate_ = (double)(track_num) / (double)(curr_camera_state->second.feature_id_set_.size());
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
        auto iter_feature_inmap = std::find_if(keypoints_featureid_.begin(), keypoints_featureid_.end(),
                                               [&](std::pair<unsigned long long int, long long int> feature_iter) {
                                                   return feature_iter.second == iter_member->first;
                                               });
        if (iter_feature_inmap == keypoints_featureid_.end())
        {
            auto &feature = iter_member->second;
            if (feature.observation_uv_.size() > 3)
            {
                if (CheckEnableTriangleate(feature) && CheckMotionStatus(feature) /*&& map_observation_set_.size() < 35*/)
                {
                    map_observation_set_.insert(*iter_member);
                }
            }
            iter_member = map_feature_set_.erase(iter_member);
            continue;
        }
        iter_member++;
    }

    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "observation map:\t" << std::endl;
    //     for (auto index : map_observation_set_)
    //     {
    //         ofs_camera_feature_log_file_ << index.first << " " << index.second << std::endl;
    //     }
    //     ofs_camera_feature_log_file_ << std::endl;
    // }
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
    Eigen::Isometry3d cami_cam0 = (cam0_trans.inverse() * cami_trans).inverse();

    auto &x1 = feature.observation_uv_.begin()->second;
    auto &x2 = feature.observation_uv_.rbegin()->second;
    Eigen::Vector3d p1(x1.x, x1.y, 1.0);
    Eigen::Vector3d p2(x2.x, x2.y, 1.0);

    Eigen::Matrix<double, 3, 2> A;
    A << cami_cam0.rotation() * p1, p2;
    const Eigen::Matrix2d ATA = A.transpose() * A;
    if (ATA.determinant() < 0.000001)
        return false;

    const Eigen::Vector2d depth2 = -ATA.inverse() * A.transpose() * cami_cam0.translation();
    double depth = fabs(depth2[0]);
    position_cam0[0] = p1(0) * depth;
    position_cam0[1] = p1(1) * depth;
    position_cam0[2] = p1(2) * depth;
    return true;
}

// bool MsckfProcess::TriangulatePoint(const Feature &feature,
//                                     const Eigen::Isometry3d &cam0_trans,
//                                     double position_cam0[3])
// {
//     const StateId &latest_id = feature.observation_uv_.rbegin()->first;

//     Eigen::Isometry3d cami_trans{Eigen::Isometry3d::Identity()};
//     cami_trans.rotate(map_state_set_[latest_id].quat_.toRotationMatrix());
//     cami_trans.pretranslate(map_state_set_[latest_id].position_);
//     Eigen::Isometry3d cam0_cami = (cam0_trans.inverse() * cami_trans);
//     // ofs_camera_feature_log_file_ << feature.feature_id_ << "  cam0_cami: \n" << std::fixed << std::setprecision(6)
//     //                              << cam0_cami.matrix() << std::endl;
//     const Eigen::Matrix4d mat_tmp = cam0_cami.matrix();
//     cv::Mat T2 = (cv::Mat_<double>(3, 4) << mat_tmp(0, 0), mat_tmp(0, 1), mat_tmp(0, 2), mat_tmp(0, 3),
//                   mat_tmp(1, 0), mat_tmp(1, 1), mat_tmp(1, 2), mat_tmp(1, 3),
//                   mat_tmp(2, 0), mat_tmp(2, 1), mat_tmp(2, 2), mat_tmp(2, 3));

//     cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
//                   0, 1, 0, 0,
//                   0, 0, 1, 0);
//     cv::Mat pts_4d;
//     std::vector<cv::Point2f> pts1, pts2;
//     pts1.emplace_back(feature.observation_uv_.begin()->second);
//     pts2.emplace_back(feature.observation_uv_.rbegin()->second);
//     cv::triangulatePoints(T1, T2, pts1, pts2, pts_4d);
//     // if (camera_feature_log_)
//     // {
//     //     ofs_camera_feature_log_file_ << feature.feature_id_ << "  T1: \n"
//     //                                  << std::fixed << std::setprecision(6)
//     //                                  << T1 << std::endl;
//     //     ofs_camera_feature_log_file_ << "  T2: \n"
//     //                                  << std::fixed << std::setprecision(6)
//     //                                  << T2 << std::endl;
//     //     ofs_camera_feature_log_file_ << pts1 << std::endl;
//     //     ofs_camera_feature_log_file_ << pts2 << std::endl;
//     //     ofs_camera_feature_log_file_ << pts_4d << std::endl;
//     // }
//     cv::Mat x = pts_4d.col(0);
//     int coeff = 1;
//     if (((pts1.at(0).x > 0 && x.at<float>(0, 0) < 0) || (pts1.at(0).x < 0 && x.at<float>(0, 0) > 0)) && x.at<float>(3, 0) > 0)
//     {
//         coeff = -1;
//     }
//     x /= (x.at<float>(3, 0) * coeff);
//     position_cam0[0] = x.at<float>(0, 0);
//     position_cam0[1] = x.at<float>(1, 0);
//     position_cam0[2] = x.at<float>(2, 0);
//     return true;
// }

/**
 * @brief  利用点的匀速变化来探测可能出现的错匹配点
 * @note   2020-02-03 增加
 * @param  &feature: 待判断的特征点
 * @retval 
 */
bool MsckfProcess::CheckMotionStatus(const Feature &feature)
{
    const auto &observation = feature.raw_uv_;
    std::vector<double> x_vector, y_vector;
    for (auto iter = observation.begin(); iter != (observation.end()); iter++)
    {
        auto iter_2 = iter;
        iter_2++;
        if (iter_2 == observation.end())
            break;
        x_vector.emplace_back(iter->second.x - iter_2->second.x);
        y_vector.emplace_back(iter->second.y - iter_2->second.y);
    }
    int size_vector = x_vector.size();
    if (size_vector < 3)
    {
        return false;
    }
    double x_average = std::accumulate(x_vector.begin(), x_vector.end(), 0) / size_vector;
    double y_average = std::accumulate(y_vector.begin(), y_vector.end(), 0) / size_vector;
    double x_std = 0.0, y_std = 0.0;
    for (size_t i = 0; i < size_vector; i++)
    {
        x_std += pow(x_vector[i] - x_average, 2) / size_vector;
        y_std += pow(y_vector[i] - y_average, 2) / size_vector;
    }
    x_std = sqrt(x_std);
    y_std = sqrt(y_std);
    // LOG(INFO) << "std:\t" << x_std << "\t" << y_std << std::endl;
    // LOG(INFO) << "average:\t" << x_average << "\t" << y_average << std::endl;
    if (x_std > 10 || y_std > 8)
    {
        return false;
    }
    for (size_t i = 0; i < size_vector; i++)
    {
        if (x_vector[i] > x_average + (x_std * 3) || x_vector[i] < x_average - (x_std * 3))
        {
            return false;
        }
        if (y_vector[i] > y_average + (y_std * 3) || y_vector[i] < y_average - (y_std * 3))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief  LM优化求解特征点在世界坐标系下的位置
 * @note   // 2020-02 测试通过
 *          逆深度参数法求解
 * @param  &feature: 待求的特征点
 * @retval 
 */
bool MsckfProcess::LMOptimizatePosition(Feature &feature)
{
    if (feature.is_initialized_)
        return true;
    ceres::Problem problem;
    Eigen::Isometry3d cam0_tranformation{Eigen::Isometry3d::Identity()};
    cam0_tranformation.rotate(map_state_set_[feature.observation_uv_.begin()->first].quat_.toRotationMatrix());
    cam0_tranformation.pretranslate(map_state_set_[feature.observation_uv_.begin()->first].position_);
    double initial_position_world[3] = {1.0, 1.0, 1.0};
    if (!TriangulatePoint(feature, cam0_tranformation, initial_position_world))
        return false;

    double position_world[3] = {initial_position_world[0] / initial_position_world[2],
                                initial_position_world[1] / initial_position_world[2],
                                1.0 / initial_position_world[2]};

    for (auto &observation : feature.observation_uv_)
    {
        auto &camera_id = observation.first;
        const auto &camera_state = map_state_set_[camera_id];
        cv::Point2f &observation_point = observation.second;
        Eigen::Isometry3d cami_tranformation{Eigen::Isometry3d::Identity()};
        cami_tranformation.rotate(camera_state.quat_.toRotationMatrix());
        cami_tranformation.pretranslate(camera_state.position_);
        Eigen::Isometry3d cam0_cami = (cam0_tranformation.inverse() * cami_tranformation);
        ceres::CostFunction *cost_function = ReprojectionError::Create(observation_point, cam0_cami);
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), position_world);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);

    feature.position_world_ << position_world[0], position_world[1], 1.0;
    feature.position_world_ *= (1 / position_world[2]);

    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "LM result: \t" << feature.feature_id_ << "\t" << feature.position_world_.transpose() << std::endl;
    //     for (auto iter = feature.raw_uv_.begin(); iter != feature.raw_uv_.end(); iter++)
    //     {
    //         const auto &camera_state_tmp = map_state_set_[iter->first];
    //         ofs_camera_feature_log_file_ << "uv_feature_pixel: \t" << camera_state_tmp.state_id_ << "\t"
    //                                      << camera_state_tmp.time_.SecondOfWeek() << "\t"
    //                                      << feature.observation_uv_.at(iter->first) << "\t"
    //                                      << iter->second << std::endl;
    //     }
    // }
    if (feature.position_world_(2) > 120 || feature.position_world_(2) < 3 || summary.num_successful_steps > 30)
    {
        // LOG(INFO) << "LM failed" << std::endl;
        // ofs_camera_feature_log_file_ << "do not used" << std::endl;
        return false;
    }
    feature.position_world_ = cam0_tranformation * feature.position_world_;

    // auto camera_state_end_iter = feature.raw_uv_.rbegin();
    // const auto &camera_state_tmp = map_state_set_[camera_state_end_iter->first];
    // Eigen::Isometry3d cam_n_tran{Eigen::Isometry3d::Identity()};
    // cam_n_tran.rotate(camera_state_tmp.quat_.toRotationMatrix());
    // cam_n_tran.pretranslate(camera_state_tmp.position_);
    // auto tmp_pos = (cam_n_tran.inverse() * feature.position_world_);
    // LOG(ERROR) <<"depth: "<<tmp_pos(2) << std::endl;
    // if (tmp_pos(2) > 50 || tmp_pos(2) < 0.5)
    //     return false;

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
    static const double fx = camera_mat_.at<double>(0, 0);
    static const double fy = camera_mat_.at<double>(1, 1);
    static const double cx = camera_mat_.at<double>(0, 2);
    static const double cy = camera_mat_.at<double>(1, 2);
    Eigen::Matrix2d J_camera_mat;
    J_camera_mat << fx, 0.0,
        0.0, fy;

    int state_count = map_state_set_.size();
    int observe_feature_count = feature.observation_uv_.size();
    const utiltool::StateIndex &state_index = filter_->GetStateIndex();

    //* 一个相机状态观测到一个feature提供两个残差[u,v]
    H_state = Eigen::MatrixXd::Zero(observe_feature_count * 2, state_count * 6 + state_index.total_state);
    z_measure = Eigen::VectorXd::Zero(observe_feature_count * 2);
    Eigen::MatrixXd H_feature = Eigen::MatrixXd::Zero(observe_feature_count * 2, 3);

    int row_index = 0;
    auto &point_coor_world = feature.position_world_;
    if (camera_feature_log_)
    {
        ofs_camera_feature_log_file_ << "measure feature position: " << feature.feature_id_ << " "
                                     << std::fixed << std::setprecision(4)
                                     << feature.position_world_.transpose() << std::endl;
    }
    for (auto member : feature.raw_uv_)
    {
        auto &camera_state = map_state_set_[member.first];
        auto &measure = member.second;

        auto iter_camera_index = state_index.camera_state_index.find(camera_state.state_id_);
        if (iter_camera_index == state_index.camera_state_index.end())
        {
            LOG(FATAL) << "find camera state index failed" << std::endl;
        }
        Eigen::Vector3d l_cf_w = (point_coor_world - camera_state.position_);
        Eigen::Vector3d point_f_c = (camera_state.quat_.toRotationMatrix().transpose()) * l_cf_w;
        Eigen::Vector2d point_f_pixel(point_f_c(0) / point_f_c(2) * fx + cx, point_f_c(1) / point_f_c(2) * fy + cy);

        Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(2, 3);
        Eigen::MatrixXd J2_1 = Eigen::MatrixXd::Zero(3, 6);
        Eigen::MatrixXd J2_2 = Eigen::MatrixXd::Zero(3, 3);

        // double z_1 = 1.0 / point_coor_world(2);
        // J1 << z_1, 0, -point_coor_world(0) * z_1 * z_1,
        //     0, z_1, -point_coor_world(1) * z_1 * z_1;

        double z_1 = 1.0 / point_f_c(2);
        J1 << z_1, 0, -point_f_c(0) * z_1 * z_1,
            0, z_1, -point_f_c(1) * z_1 * z_1;

        J1 = J_camera_mat * J1;
        J2_1.block<3, 3>(0, 3) = -1 * camera_state.quat_.toRotationMatrix().transpose() *
                                 utiltool::skew(l_cf_w); //* 相机姿态系数

        J2_1.block<3, 3>(0, 0) = -1 * camera_state.quat_.toRotationMatrix().transpose(); //* 相机位置系数

        J2_2 = camera_state.quat_.toRotationMatrix().transpose(); //* 特征点系数
        H_feature.block<2, 3>(row_index, 0) = J1 * J2_2;
        H_state.block<2, 6>(row_index, iter_camera_index->second) = J1 * J2_1;
        z_measure.segment<2>(row_index) << point_f_pixel(0) - measure.x,
            point_f_pixel(1) - measure.y;
        row_index += 2;
        // if (camera_feature_log_)
        // {
        //     ofs_camera_feature_log_file_
        //         << "camera_state_position: " << std::fixed << std::setprecision(10) << camera_state.position_.transpose() << std::endl
        //         << "camera_state_rotation: " << std::endl
        //         << camera_state.quat_.toRotationMatrix() << std::endl
        //         << "measure uv: " << std::fixed << std::setprecision(10) << measure << std::endl
        //         << "point_f_c: " << point_f_c.transpose() << std::endl
        //         << "l_cf_w: " << l_cf_w.transpose() << std::endl
        //         << "J1: " << std::endl
        //         << J1 << std::endl
        //         << "J2_1: " << std::endl
        //         << J2_1 << std::endl
        //         << "J2_2: " << std::endl
        //         << J2_2 << std::endl
        //         << "H_state: " << std::endl
        //         << H_state << std::endl
        //         << "H_feature: " << std::endl
        //         << H_feature << std::endl
        //         << "z_measure: " << std::endl
        //         << z_measure << std::endl;
        // }
    }
    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "Hfeature:" << std::endl
    //                                  << std::setprecision(8) << H_feature << std::endl;
    //     ofs_camera_feature_log_file_ << "Hstate:" << std::endl
    //                                  << std::setprecision(8) << H_state << std::endl;
    //     ofs_camera_feature_log_file_ << "z_measure:" << std::endl
    //                                  << std::setprecision(8) << z_measure << std::endl;
    // }
    //* SVD求解Hf的左零空间，消除Hf
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(H_feature, Eigen::ComputeFullU | Eigen::ComputeThinV);
    Eigen::MatrixXd A = svd_helper.matrixU().rightCols(observe_feature_count * 2 - 3);
    H_state = A.transpose() * H_state;
    z_measure = A.transpose() * z_measure;
    // if (camera_feature_log_)
    // {
    //     ofs_camera_feature_log_file_ << "AT:" << std::endl
    //                                  << std::setprecision(8) << A.transpose() << std::endl;
    //     ofs_camera_feature_log_file_ << "Hstate2:" << std::endl
    //                                  << std::setprecision(8) << H_state << std::endl;
    //     ofs_camera_feature_log_file_ << "z_measure2:" << std::endl
    //                                  << std::setprecision(8) << z_measure << std::endl;
    // }
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

    Eigen::MatrixXd H_thin;
    Eigen::VectorXd Z_thin;
    if (H_state.rows() > H_state.cols()) //TODO待更新为QR后进行量测
    {
        Eigen::HouseholderQR<Eigen::MatrixXd> qr;
        qr.compute(H_state);
        Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
        Eigen::MatrixXd Q = qr.householderQ();
        // ofs_camera_feature_log_file_ << std::fixed << std::setprecision(12) << "R: " << std::endl
        //                              << R << std::endl;
        Eigen::VectorXd r_temp;
        r_temp = Q.transpose() * z_measure;
        // ofs_camera_feature_log_file_
        //     << std::fixed << std::setprecision(12) << "H_state: " << std::endl
        //     << H_state << std::endl;
        // ofs_camera_feature_log_file_ << std::fixed << std::setprecision(12) << "z_measure: " << std::endl
        //                              << z_measure << std::endl;
        // ofs_camera_feature_log_file_ << std::fixed << std::setprecision(12) << "H_temp: " << std::endl
        //                              << H_temp << std::endl;
        // ofs_camera_feature_log_file_ << std::fixed << std::setprecision(12) << "r_temp: " << std::endl
        //                              << r_temp << std::endl;
        // ofs_camera_feature_log_file_.flush();
        H_thin = R.topRows(H_state.cols());
        Z_thin = r_temp.head(H_state.cols());
    }
    else
    {
        H_thin = H_state;
        Z_thin = z_measure;
    }

    Eigen::MatrixXd measure_noise = Eigen::MatrixXd::Identity(H_thin.rows(), H_thin.rows());
    measure_noise = measure_noise * (pixel_noise * pixel_noise);
    return filter_->MeasureUpdate(H_thin, Z_thin, measure_noise, curr_time_);
}

/**
 * @brief  对相机状态进行反馈，更新相机的位姿
 * @note   
 * @param  &dx_camera: 
 * @retval None
 */
void MsckfProcess::ReviseCameraState(const Eigen::VectorXd &dx_camera)
{
    // static std::string result_path_out = config_->get<std::string>("result_output_path") + "/camera_state_log.log";
    // static std::ofstream ofs_camera_state_log(result_path_out);
    // ofs_camera_state_log << "curr_state_time: " << std::fixed << std::setprecision(4)
    //                      << curr_time_.SecondOfWeek() << "\t";

    int camera_state_count = 0;
    // map_state_set_.size();
    assert(dx_camera.size() == map_state_set_.size() * 6);
    for (auto &camera_state : map_state_set_)
    {
        Eigen::Vector3d t_camera = dx_camera.segment<3>(camera_state_count * 6);
        Eigen::Vector3d theta_camera = dx_camera.segment<3>(camera_state_count * 6 + 3);
        auto camera_quat_delta = RotationVector2Quaternion(theta_camera);
        camera_state.second.quat_ = camera_quat_delta * camera_state.second.quat_;
        camera_state.second.position_ -= t_camera;
        camera_state.second.quat_.normalize();
        camera_state_count++;

        // auto BLH = earth::WGS84XYZ2BLH(camera_state.second.position_);
        // ofs_camera_state_log << std::fixed << std::setprecision(4)
        //                      << camera_state.second.time_.SecondOfWeek() << "\t"
        //                      << camera_state.first << "\t"
        //                      << camera_state.second.position_.transpose() << "\t"
        //                      << constant::rad2deg *
        //                             attitude::RotationMartix2Euler(
        //                                 earth::CalCe2n(BLH(0), BLH(1)) *
        //                                 camera_state.second.quat_.toRotationMatrix() * cam_imu_tranformation_.rotation().transpose())
        //                                 .transpose()
        //                      << "\t";
    }
    // ofs_camera_state_log << std::endl;
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
            // ofs_camera_feature_log_file_ << std::fixed << std::setprecision(8) << "iter_camera->first: " << iter_camera->first
            //                              << "\t iter_index_camera: " << (iter_index_camera->second) << std::endl;
            // ofs_camera_feature_log_file_ << "state_cov before: " << std::endl
            //                              << filter_->GetStateCov() << std::endl;
            filter_->EliminateIndex(iter_index_camera->second, 6);
            // ofs_camera_feature_log_file_ << "state_cov after: " << std::endl
            //                              << filter_->GetStateCov() << std::endl;
            // ofs_camera_feature_log_file_ << "index.camera_state_index before: " << index.camera_state_index << std::endl;
            iter_index_camera = index.camera_state_index.erase(iter_index_camera);
            for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
            {
                iter_index_camera->second -= 6;
            }
            // ofs_camera_feature_log_file_ << "index.camera_state_index after: " << index.camera_state_index << std::endl;
            // ofs_camera_feature_log_file_ << "map_state_set_ before: " << map_state_set_ << std::endl;
            iter_camera = map_state_set_.erase(iter_camera);
            // ofs_camera_feature_log_file_ << "map_state_set_ after: " << map_state_set_ << std::endl;
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
    int count = 0;
    const auto &index = filter_->GetStateIndex();
    Eigen::MatrixXd Hstate_all;
    Eigen::VectorXd Zmeasure_all;
    for (auto iter_feature = map_observation_set_.begin(); iter_feature != map_observation_set_.end(); iter_feature++)
    {
        bool temp = LMOptimizatePosition(iter_feature->second);
        if (temp)
        {
            Eigen::MatrixXd H_state;
            Eigen::VectorXd z_measure;
            iter_feature->second.is_initialized_ = true;

            MeasurementJacobian(iter_feature->second, H_state, z_measure);
            if (GatingTest(H_state, z_measure, z_measure.rows()))
            {
                Hstate_all.conservativeResize(H_state.rows() + Hstate_all.rows(), H_state.cols());
                Hstate_all.bottomRows(H_state.rows()) = H_state;

                Zmeasure_all.conservativeResize(z_measure.rows() + Zmeasure_all.rows(), 1);
                Zmeasure_all.bottomRows(z_measure.rows()) = z_measure;
                count++;
                if (Hstate_all.rows() > 600)
                    break;
            }
            else
            {
                if (camera_feature_log_)
                {
                    ofs_camera_feature_log_file_ << "GatingTest failed: "
                                                 << iter_feature->second.feature_id_ << "\t"
                                                 << iter_feature->second.position_world_.transpose() << std::endl;
                }
            }
        }
    }
    if (Hstate_all.rows() < 20 || count < 3)
        return;
    LOG(INFO) << "Measure Feature Number is " << count << std::endl;

    Eigen::VectorXd dx = MeasurementUpdate(Hstate_all, Zmeasure_all);

    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/dx.log");
    // static std::ofstream ofs_dx_log(output_path);
    // ofs_dx_log << std::fixed << std::setprecision(3) << navinfo.time_.SecondOfWeek() << " "
    //            << std::fixed << std::setprecision(8) << dx.segment<9>(index.pos_index_).transpose() << "\t"
    //            << dx.tail(6).transpose() << std::endl;

    if (dx.segment<3>(index.pos_index_).norm() > 1.0 || dx.segment<3>(index.vel_index_).norm() > 1.0)
    {
        LOG(ERROR) << "The value of resive dx is not correct, time: "
                     << navinfo.time_.SecondOfWeek()
                     << "\tpos: " << dx.segment<3>(index.pos_index_).transpose()
                     << "\tvel: " << dx.segment<3>(index.vel_index_).transpose() << std::endl;
        return;
    }
    Eigen::VectorXd dx_imu = dx.head(filter_->GetStateIndex().total_state);

    filter_->ReviseState(navinfo, dx_imu);
    ReviseCameraState(dx.tail(dx.size() - filter_->GetStateIndex().total_state));
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

        if (angle < 0.2618 && distance < 0.4 /*&& tracking_rate_ > 0.5*/)
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
                tmp_feature.raw_uv_[rm_cam_state_ids[0]] = feature.raw_uv_[iter_cam1->first];
                tmp_feature.raw_uv_[rm_cam_state_ids[1]] = feature.raw_uv_[iter_cam2->first];
                tmp_feature.is_initialized_ = true;
                tmp_feature.position_world_ = feature.position_world_;
                feature.is_initialized_ = false;
                map_observation_set_[tmp_feature.feature_id_] = tmp_feature;
            }
            auto camera_id = iter_cam1->first;
            feature.observation_uv_.erase(iter_cam1);
            feature.raw_uv_.erase(camera_id);

            camera_id = iter_cam2->first;
            feature.observation_uv_.erase(iter_cam2);
            feature.raw_uv_.erase(camera_id);
            continue;
        }
        if (iter_cam1 != feature.observation_uv_.end())
        {
            auto camera_id = iter_cam1->first;
            feature.observation_uv_.erase(iter_cam1);
            feature.raw_uv_.erase(camera_id);
        }
        if (iter_cam2 != feature.observation_uv_.end())
        {
            auto camera_id = iter_cam2->first;
            feature.observation_uv_.erase(iter_cam2);
            feature.raw_uv_.erase(camera_id);
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
        // ofs_camera_feature_log_file_ << "iter_camera->first: (camera_remove)" << iter_camera
        //                              << "\t iter_index_camera: " << (iter_index_camera->second) << std::endl;
        // ofs_camera_feature_log_file_ << "state_cov before: " << std::endl
        //                              << filter_->GetStateCov() << std::endl;
        filter_->EliminateIndex(iter_index_camera->second, 6);
        // ofs_camera_feature_log_file_ << "state_cov after: " << std::endl
        //                              << filter_->GetStateCov() << std::endl;
        // ofs_camera_feature_log_file_ << "index.camera_state_index before: " << index.camera_state_index << std::endl;

        iter_index_camera = index.camera_state_index.erase(iter_index_camera);
        for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
        {
            iter_index_camera->second -= 6;
        }
        // ofs_camera_feature_log_file_ << "index.camera_state_index after: " << index.camera_state_index << std::endl;
        // ofs_camera_feature_log_file_ << "map_state_set_ before: " << map_state_set_ << std::endl;
        map_state_set_.erase(iter_camera);
        // ofs_camera_feature_log_file_ << "map_state_set_ after: " << map_state_set_ << std::endl;
    }
}

/**
 * @brief  卡方检验 摘自msckf:https://github.com/KumarRobotics/msckf_vio
 * @note   
 * @param  &H: 
 * @param  &r: 
 * @param  &dof: 
 * @retval 
 */
bool MsckfProcess::GatingTest(const Eigen::MatrixXd &H, const Eigen::VectorXd &r, const int &dof)
{
    static double pixel_noise = config_->get<double>("camera_pixel_noise");
    const auto &state_cov = filter_->GetStateCov();
    Eigen::MatrixXd P1 = H * state_cov * H.transpose();
    Eigen::MatrixXd P2 = pixel_noise * pixel_noise * Eigen::MatrixXd::Identity(H.rows(), H.rows());
    double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);
    double average = 0;
    for (int i = 0; i < r.rows(); ++i)
    {
        average += fabs(r(i)) / r.rows();
        if (fabs(r(i)) > 20)
        {
            average = 20;
            break;
        }
    }
    if (gamma < utiltool::chi_squared_test_table[dof - 1] && average < 10)
    {
        return true;
    }
    else
    {
        LOG(INFO) <<std::fixed <<std::setprecision(3) <<curr_time_.SecondOfWeek() << "\tgamma: " << gamma << "\tdof:" << dof << "\t average" << average << std::endl;
        return false;
    }
}

void MsckfProcess::NormKeyPoints(const std::vector<cv::Point2f> &keypoint_distorted,
                                 std::vector<cv::Point2f> &keypoint_undistorted,
                                 const cv::Mat &camera_mat_)
{
    cv::Point2f tmp_key;
    const double &fx = camera_mat_.at<double>(0, 0);
    const double &fy = camera_mat_.at<double>(1, 1);
    const double &cx = camera_mat_.at<double>(0, 2);
    const double &cy = camera_mat_.at<double>(1, 2);
    for (auto &member : keypoint_distorted)
    {
        tmp_key = member;
        tmp_key.x = (member.x - cx) / fx;
        tmp_key.y = (member.y - cy) / fy;
        keypoint_undistorted.emplace_back(tmp_key);
    }
}

void MsckfProcess::Test()
{
    auto curr_camera_state = map_state_set_.end();
    curr_camera_state--;
    auto pre_camera_state = curr_camera_state;
    pre_camera_state--;

    cv::Point2d principal_point(332.8951, 229.7975); //相机光心，TUM dataset 的标定值
    double focal_length = 1155.2;                    //相机焦距, TUM dataset标定值
    cv::Mat essential_mat = findEssentialMat(curr_frame_keypoints_, pre_frame_keypoints_, focal_length, principal_point);

    cv::Mat R, t;
    cv::recoverPose(essential_mat, curr_frame_keypoints_, pre_frame_keypoints_, R, t, focal_length, principal_point);

    Eigen::Matrix3d R_camera;
    Eigen::Vector3d t_camera;
    cv::cv2eigen(R, R_camera);
    cv::cv2eigen(t, t_camera);

    Eigen::Matrix3d R_imu1 = pre_camera_state->second.quat_.toRotationMatrix() * cam_imu_tranformation_.rotation().transpose();
    Eigen::Matrix3d R_imu2 = curr_camera_state->second.quat_.toRotationMatrix() * cam_imu_tranformation_.rotation().transpose();
    Eigen::Vector3d Rimu12 = attitude::RotationMartix2Euler(R_imu1.transpose() * R_imu2);
    auto rotation_imu = attitude::Quaternion2Euler(pre_camera_state->second.quat_.conjugate() * curr_camera_state->second.quat_);
    auto rotation_cam = attitude::RotationMartix2Euler(R_camera);
    auto t_imu = pre_camera_state->second.quat_.conjugate() * (curr_camera_state->second.position_ - pre_camera_state->second.position_);
    ofs_camera_feature_log_file_ << "camera_imu:" << curr_time_.SecondOfWeek() << " " << curr_camera_state->second.time_.SecondOfWeek() << " " << std::fixed
                                 << std::setprecision(8) << rotation_imu.transpose() * constant::rad2deg << " "
                                 << rotation_cam.transpose() * constant::rad2deg << " "
                                 << Rimu12.transpose() * constant::rad2deg << " "
                                 << t_imu.transpose() * 100 << "  "
                                 << t_camera.transpose() * 100 << "  "
                                 << std::endl;
}

} // namespace camera

} // namespace mscnav