/*
** msckf.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午8:36:30 2019 little fang
** Last update Sat Apr 24 下午3:40:01 2020 little fang
*/

#include "navattitude.hpp"
#include "camera/msckf.hpp"
#include "camera/imageprocess.h"
#include "navbase.hpp"
#include <navearth.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/videoio/videoio.hpp>

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
    cam_imu_tranformation_.linear() = (rotation);
    cam_imu_tranformation_.translation() = (translation);
    LOG(INFO) << "cam_imu: \n"
              << rotation << std::endl;
    auto att = attitude::RotationMartix2Euler(rotation);
    LOG(INFO) << att.transpose() * constant::rad2deg << std::endl;
    LOG(INFO) << translation.transpose() << std::endl;

    //** 相关日志记录输出
    camera_state_log_enable_ = (config_->get<int>("camera_state_log_enable") == 0) ? false : true;
    msckf_info_log_enable_ = (config_->get<int>("msckf_info_log_enable") == 0) ? false : true;
    debug_info_log_enable_ = (config_->get<int>("debug_info_log_enable") == 0) ? false : true;
    feature_log_enable_ = (config_->get<int>("feature_info_log_enable") == 0) ? false : true;
    utiltool::NavTime time = utiltool::NavTime::NowTime();
    std::string result_path = config_->get<std::string>("result_output_path");
    std::string log_append = time.Time2String("%04d-%02d-%02d-%02d-%02d-%.1f");
    std::string out_filepath;
    if (camera_state_log_enable_) // 记录全部的相机状态信息，预测值/更新值等
    {
        out_filepath = result_path + ("/camera_state.log-" + log_append);
        ofs_camera_state_log_file_.open(out_filepath);
        if (!ofs_camera_state_log_file_.good())
        {
            LOG(ERROR) << "Camera state log file open failed. Path: " << out_filepath << std::endl;
            getchar();
        }
    }
    if (feature_log_enable_) // 记录全部feature信息包括三角化结果，是否被使用等
    {
        out_filepath = result_path + ("/feature_info.log-" + log_append);
        ofs_feature_log_file_.open(out_filepath);
        if (!ofs_feature_log_file_.good())
        {
            LOG(ERROR) << "Feature log file open failed. Path: " << out_filepath << std::endl;
            getchar();
        }
    }
    if (debug_info_log_enable_) //记录大量中间调试信息
    {
        out_filepath = result_path + ("/debug_info.log-" + log_append);
        ofs_debug_info_log_file_.open(out_filepath);
        if (!ofs_debug_info_log_file_.good())
        {
            LOG(ERROR) << "Debug log file open failed. Path: " << out_filepath << std::endl;
            getchar();
        }
        ofs_debug_info_log_file_ << std::fixed << std::setprecision(8) << "camera_intrinsic:" << std::endl
                                 << camera_mat_ << std::endl;

        ofs_debug_info_log_file_ << std::fixed << std::setprecision(8) << "camera_distcoeffs:" << std::endl
                                 << dist_coeffs_ << std::endl;

        ofs_debug_info_log_file_ << std::fixed << std::setprecision(8) << "camera_imu_rotation:" << std::endl
                                 << cam_imu_tranformation_.rotation() << std::endl;

        ofs_debug_info_log_file_ << std::fixed << std::setprecision(8) << "camera_imu_translation:"
                                 << cam_imu_tranformation_.translation().transpose() << std::endl;
    }
    if (msckf_info_log_enable_) //msckf处理日志信息，错误警告等信息
    {
        out_filepath = result_path + ("/msckf_info.log-" + log_append);
        ofs_msckf_info_log_file_.open(out_filepath);
        if (!ofs_msckf_info_log_file_.good())
        {
            LOG(ERROR) << "Msckf log file open failed. Path: " << out_filepath << std::endl;
            getchar();
        }
    }
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

void DrawsMatches(const cv::Mat &img1,
                  const cv::Mat &img2,
                  const std::vector<cv::Point2f> &point1,
                  const std::vector<cv::Point2f> &point2)
{
    assert(point1.size() == point2.size());
    int rows = img1.rows > img2.rows ? img1.rows : img2.rows;
    int cols = img1.cols + img2.cols;
    cv::Mat image(rows, cols, img1.type(), cv::Scalar(0));
    img1.copyTo(image.colRange(0, img1.cols));
    img2.copyTo(image.colRange(img1.cols, cols));
    for (size_t i = 0; i < point1.size(); i++)
    {
        cv::RNG rng(i);
        cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::Point2f temp(point2[i].x + img1.cols, point2[i].y);
        cv::line(image, point1[i], temp, color, 1);
        cv::circle(image, point1[i], 3.5, color, 1);
        cv::circle(image, temp, 3.5, color, 1);
    }
    std::string info = "matches size: " + std::to_string(point1.size());
    cv::putText(image, info.c_str(), cv::Point(40, 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar::all(-1));
    cv::imshow("matches", image);
    cv::waitKey(1);
}

bool MsckfProcess::ProcessImage(const cv::Mat &img_raw, const utiltool::NavTime &time, utiltool::NavInfo &navinfo)
{
    static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/log.avi");
    static cv::VideoWriter video(output_path, CV_FOURCC('X', 'V', 'I', 'D'), 30.0, img_raw.size(), false);

    cv::Mat img1 = img_raw.clone();
    // cv::equalizeHist(img_raw, img1);
    // cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    // cv::filter2D(img1, img1, CV_8U, kernel);

    curr_time_ = time;
    if (is_first_)
    {
        FirstImageProcess(img1, navinfo);
        cv::swap(pre_img, img1);
        is_first_ = false;
        return false;
    }

    //预测点
    // auto pre_camera_state = map_state_set_.rbegin();
    // Eigen::Matrix3d curr_rotation = navinfo.rotation_ * cam_imu_tranformation_.rotation();
    // Eigen::Matrix3d rotation_imu = (curr_rotation * pre_camera_state->second.quat_.toRotationMatrix());
    // cv::Mat R_p_c;
    // cv::eigen2cv(rotation_imu, R_p_c);
    // ImageProcess::PredictFeatureTracking(pre_frame_keypoints_, R_p_c, camera_mat_, curr_frame_keypoints_);

    // 光流追踪
    ImageProcess::LKTrack(pre_img, img1, keypoints_id_, pre_frame_keypoints_, curr_frame_keypoints_);

    if (curr_frame_keypoints_.size() == 0)
    {
        LOG(ERROR) << std::fixed << std::setprecision(3) << curr_time_.SecondOfWeek() << ":LK Failed" << std::endl;
        return false;
    }

    if (CheckStaticMotion())
    {
        LOG(ERROR) << std::fixed << std::setprecision(3) << curr_time_.SecondOfWeek() << ": Static Motion" << std::endl;
        return false;
    }
    // DrawsMatches(pre_img, img1, pre_frame_keypoints_, curr_frame_keypoints_);
    //** 增加当前camera State到系统中去.
    AguementedState(navinfo);

    // 外点剔除
    // LOG(INFO) << map_state_set_.size() << std::endl;
    auto curr_camera_state = map_state_set_.rbegin();
    auto pre_camera_state = curr_camera_state;
    pre_camera_state++;
    Eigen::Matrix3d rotation_imu = attitude::Quaternion2RotationMatrix(
        curr_camera_state->second.quat_.conjugate() * pre_camera_state->second.quat_);
    Eigen::Vector3d t = curr_camera_state->second.position_ - pre_camera_state->second.position_;
    t = curr_camera_state->second.quat_.conjugate().toRotationMatrix() * t;
    // ImageProcess::CheckFeatureTrack(pre_frame_keypoints_,
    //                                 curr_frame_keypoints_,
    //                                 keypoints_id_,
    //                                 rotation_imu,
    //                                 t,
    //                                 dist_coeffs_,
    //                                 camera_mat_);
    // cv::Mat R_c;
    // cv::eigen2cv(rotation_imu, R_c);

    // ImageProcess::TwoPointRansac(pre_frame_keypoints_,
    //                              curr_frame_keypoints_,
    //                              R_c,
    //                              camera_mat_,
    //                              dist_coeffs_,
    //                              2,
    //                              0.99,
    //                              inlier_markers);
    // assert(inlier_markers.size() == pre_frame_keypoints_.size());
    // int k = 0, count_size = curr_frame_keypoints_.size();
    // for (size_t i = 0; i < count_size; i++)
    // {
    //     if (inlier_markers[i] == 1)
    //     {
    //         pre_frame_keypoints_[k] = pre_frame_keypoints_[i];
    //         curr_frame_keypoints_[k] = curr_frame_keypoints_[i];
    //         keypoints_id_[k] = keypoints_id_[i];
    //         ++k;
    //     }
    // }
    // keypoints_id_.resize(k);
    // curr_frame_keypoints_.resize(k);
    // pre_frame_keypoints_.resize(k);

    cv::Mat key_image = img1.clone();
    for (int i = 0; i < curr_frame_keypoints_.size(); i++)
    {
        circle(key_image, curr_frame_keypoints_[i], 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    AddObservation();

    std::string info = "point number: " + std::to_string(curr_frame_keypoints_.size()) + "  track_rate: " + std::to_string(tracking_rate_);
    cv::putText(key_image, info.c_str(), cv::Point(40, key_image.rows - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar::all(-1));
    info = curr_time_.Time2String("%d %.3f", utiltool::NavTime::GPSTIME);
    cv::putText(key_image, info.c_str(), cv::Point(40, key_image.rows - 60), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar::all(-1));
    cv::imshow("key_image", key_image);
    cv::waitKey(1);
    video << key_image;
    // Test();

    //** 匹配的特征加入观测序列中

    //** 选择本次参与量测更新的点集
    DetermineMeasureFeature();

    //** 特征点量测更新
    FeatureMeasureUpdate(navinfo);

    //** 清除已经完成全部feature点量测的camera state
    RemoveCameraState();

    // if (map_state_set_.size() > max_camera_size)
    // {
    //     RemoveRedundantCamStates(navinfo);
    // }

    ImageProcess::GoodFreatureDetect(img1, keypoints_id_, curr_frame_keypoints_);

    std::swap(pre_frame_keypoints_, curr_frame_keypoints_);
    cv::swap(pre_img, img1);
    // if (map_state_set_.size() > 0)
    // {
    //     auto iter = map_state_set_.rbegin();
    //     navinfo.pos_camera = iter->second.position_;
    //     auto BLH = earth::WGS84XYZ2BLH(navinfo.pos_);
    //     auto rotation = cam_imu_tranformation_.rotation().transpose();
    //     navinfo.att_camera = attitude::RotationMartix2Euler(earth::CalCe2n(BLH(0), BLH(1)) * iter->second.quat_.toRotationMatrix() * rotation);
    // }
    // else
    // {
    //     LOG(ERROR) << "reset state" << std::endl;
    // }
    if (map_state_set_.size() == 0)
    {
        LOG(ERROR) << "Track Lost..." << std::endl;
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

    cam_state.position_ = navinfo.pos_ + navinfo.rotation_ * cam_imu_tranformation_.translation();
    cam_state.quat_ = attitude::RotationMartix2Quaternion(navinfo.rotation_ * cam_imu_tranformation_.rotation());
    cam_state.time_ = navinfo.time_;
    map_state_set_.insert(std::make_pair(cam_state.state_id_, cam_state));
    // map_state_set_[cam_state.state_id_] = cam_state;//!不要采用这种方式赋值，id号会不对

    //* 增广对应状态的方差信息到滤波的方程协方差矩阵中
    utiltool::StateIndex &index = filter_->GetStateIndex();
    int state_count = index.total_state + index.camera_state_index.size() * 6;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(state_count + 6, state_count);
    J.block(0, 0, state_count, state_count) = Eigen::MatrixXd::Identity(state_count, state_count);

    J.block<3, 3>(state_count + 3, index.att_index_) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(state_count, index.att_index_) = utiltool::skew(navinfo.rotation_ * cam_imu_tranformation_.translation());
    J.block<3, 3>(state_count, index.pos_index_) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd &state_cov = filter_->GetStateCov();

    state_cov = J * state_cov * J.transpose();

    int the_latest_count_state = (index.total_state) + index.camera_state_index.size() * 6;
    index.camera_state_index[cam_state.state_id_] = the_latest_count_state;

    if (camera_state_log_enable_)
    {
        ofs_camera_state_log_file_ << std::setprecision(3) << navinfo.time_.SecondOfWeek() << "  ";
        ofs_camera_state_log_file_ << std::setprecision(4) << navinfo.pos_.transpose() << " ";
        ofs_camera_state_log_file_ << std::setprecision(4) << navinfo.att_.transpose() * constant::rad2deg << " ";
        ofs_camera_state_log_file_ << std::setprecision(8) << navinfo.quat_.coeffs().transpose() << std::endl;

        auto BLH = earth::WGS84XYZ2BLH(cam_state.position_);
        auto att = attitude::RotationMartix2Euler(earth::CalCe2n(BLH(0), BLH(1)) * cam_state.quat_.toRotationMatrix());

        ofs_camera_state_log_file_ << std::setprecision(4) << cam_state.position_.transpose() << " ";
        ofs_camera_state_log_file_ << std::setprecision(4) << att.transpose() * constant::rad2deg << " ";
        ofs_camera_state_log_file_ << std::setprecision(8) << cam_state.quat_.coeffs().transpose() << std::endl;

        ofs_camera_state_log_file_ << "map camera: " << map_state_set_ << std::endl;
        ofs_camera_state_log_file_ << "camera state index: " << index.camera_state_index << std::endl
                                   << std::endl;
        ofs_camera_state_log_file_ << J << std::endl
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

//         if (feature_log_enable_)
//         {
//             ofs_feature_log_file_ << "feature map:\t" << std::endl;
//             for (auto index : map_feature_set_)
//             {
//                 ofs_feature_log_file_ << index.first << " " << index.second << std::endl;
//             }
//             ofs_feature_log_file_ << std::endl;
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
    auto curr_camera_state = map_state_set_.rbegin();
    auto pre_camera_state = curr_camera_state;
    pre_camera_state++;

    int track_num = 0;
    int size = keypoints_id_.size();
    assert(size == curr_frame_keypoints_.size());
    assert(size == pre_frame_keypoints_.size());

    std::vector<cv::Point2f> keypoint_distorted, keypoint_undistorted;
    for (size_t i = 0; i < size; i++)
    {
        // if (inlier_markers.at(i) == 0)
        //     continue;
        auto iter_point_id = keypoints_featureid_.find(keypoints_id_.at(i));
        if (iter_point_id == keypoints_featureid_.end())
        {
            Feature feature;
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.emplace_back(pre_frame_keypoints_.at(i));
            keypoint_distorted.emplace_back(curr_frame_keypoints_.at(i));

            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            feature.observation_uv_[pre_camera_state->first] = keypoint_undistorted[0];
            feature.observation_uv_[curr_camera_state->first] = keypoint_undistorted[1];

            cv::undistortPoints(keypoint_distorted, keypoint_undistorted,
                                camera_mat_, dist_coeffs_, cv::noArray(), camera_mat_);
            feature.raw_uv_[pre_camera_state->first] = keypoint_undistorted[0];
            feature.raw_uv_[curr_camera_state->first] = keypoint_undistorted[1];

            map_feature_set_.insert(std::make_pair(feature.feature_id_, feature));
            // map_feature_set_[feature.feature_id_] = feature;
            curr_trainidx_feature_map_[keypoints_id_.at(i)] = feature.feature_id_;
            curr_camera_state->second.feature_id_set_.emplace_back(feature.feature_id_);
            pre_camera_state->second.feature_id_set_.emplace_back(feature.feature_id_);
            if (feature_log_enable_)
            {
                ofs_feature_log_file_ << "new feature: " << feature.feature_id_ << " "
                                      << keypoint_undistorted[1] << "  "
                                      << keypoints_id_.at(i) << std::endl;
            }
        }
        else
        {
            keypoint_distorted.clear();
            keypoint_undistorted.clear();
            keypoint_distorted.emplace_back(curr_frame_keypoints_.at(i));
            cv::undistortPoints(keypoint_distorted, keypoint_undistorted, camera_mat_, dist_coeffs_);
            map_feature_set_[iter_point_id->second].observation_uv_[curr_camera_state->first] = keypoint_undistorted[0];

            cv::undistortPoints(keypoint_distorted, keypoint_undistorted,
                                camera_mat_, dist_coeffs_, cv::noArray(), camera_mat_);
            map_feature_set_[iter_point_id->second].raw_uv_[curr_camera_state->first] = keypoint_undistorted[0];

            curr_trainidx_feature_map_[keypoints_id_.at(i)] = iter_point_id->second;
            curr_camera_state->second.feature_id_set_.emplace_back(iter_point_id->second);
            track_num++;
            if (feature_log_enable_)
            {
                ofs_feature_log_file_ << "add feature: " << iter_point_id->second << " "
                                      << keypoint_undistorted[0] << "  "
                                      << keypoints_id_.at(i) << std::endl;
            }
        }
    }
    keypoints_featureid_ = curr_trainidx_feature_map_;
    if (feature_log_enable_)
    {
        ofs_feature_log_file_ << std::fixed << std::setprecision(3) << curr_time_.SecondOfWeek()
                              << "feature map:" << std::endl;
        for (auto index : map_feature_set_)
        {
            ofs_feature_log_file_ << index.first << " " << index.second << std::endl;
        }
        ofs_feature_log_file_ << std::endl
                              << std::endl
                              << "keypoints_featureid_:" << std::endl;
        for (auto index : keypoints_featureid_)
        {
            ofs_feature_log_file_ << index.first << " " << index.second << std::endl;
        }
        ofs_feature_log_file_ << std::endl
                              << std::endl;
    }

    tracking_rate_ = (double)(track_num) / (double)(curr_camera_state->second.feature_id_set_.size());
    LOG(INFO) << "Track rate: " << std::fixed << std::setprecision(3) << tracking_rate_ * 100 << "%" << std::endl;
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
            if (feature.observation_uv_.size() > 5)
            {
                if (CheckEnableTriangleate(feature) && CheckMotionStatus(feature) /*&& map_observation_set_.size() < 35*/)
                {
                    map_observation_set_.insert(*iter_member);
                }
            }
            for (auto &iter : map_state_set_)
            {
                auto &feature_id_vector = iter.second.feature_id_set_;
                std::remove(std::begin(feature_id_vector), std::end(feature_id_vector), iter_member->first);
            }
            iter_member = map_feature_set_.erase(iter_member);

            continue;
        }
        iter_member++;
    }

    if (feature_log_enable_)
    {
        ofs_feature_log_file_ << "observation map: " << std::endl;
        for (auto index : map_observation_set_)
        {
            ofs_feature_log_file_ << index.first << " " << index.second << std::endl;
        }
        ofs_feature_log_file_ << std::endl;
    }
    static int max_camera_size = config_->get<int>("max_camera_sliding_window");
    LOG_FIRST_N(INFO, 1) << "max_camera_size: " << max_camera_size << std::endl;
    if (map_state_set_.size() > max_camera_size)
    {
        RemoveRedundantCamStates();
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
    if (debug_info_log_enable_)
    {
        ofs_debug_info_log_file_ << "CheckEnableTriangleate: " << feature.feature_id_ << " "
                                 << orthogonal_translation.norm() << std::endl;
    }
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
    cami_trans.linear() = (map_state_set_[latest_id].quat_.toRotationMatrix());
    cami_trans.translation() = (map_state_set_[latest_id].position_);
    Eigen::Isometry3d cami_cam0 = (cami_trans.inverse() * cam0_trans);

    auto &x1 = feature.observation_uv_.begin()->second;
    auto &x2 = feature.observation_uv_.rbegin()->second;
    Eigen::Vector3d p1(x1.x, x1.y, 1.0);
    Eigen::Vector3d p2(x2.x, x2.y, 1.0);

    Eigen::Matrix<double, 3, 2> A;
    A << cami_cam0.rotation() * p1, p2;
    const Eigen::Matrix2d ATA = A.transpose() * A;
    if (ATA.determinant() < 0.00001)
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
//     // ofs_feature_log_file_ << feature.feature_id_ << "  cam0_cami: \n" << std::fixed << std::setprecision(6)
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
//     // if (feature_log_enable_)
//     // {
//     //     ofs_feature_log_file_ << feature.feature_id_ << "  T1: \n"
//     //                                  << std::fixed << std::setprecision(6)
//     //                                  << T1 << std::endl;
//     //     ofs_feature_log_file_ << "  T2: \n"
//     //                                  << std::fixed << std::setprecision(6)
//     //                                  << T2 << std::endl;
//     //     ofs_feature_log_file_ << pts1 << std::endl;
//     //     ofs_feature_log_file_ << pts2 << std::endl;
//     //     ofs_feature_log_file_ << pts_4d << std::endl;
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

    if (debug_info_log_enable_)
    {
        ofs_debug_info_log_file_ << "CheckMotionStatus: " << feature.feature_id_ << " "
                                 << x_std << " " << y_std << std::endl;
    }

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
    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/ceres_lm_in.log");
    // static std::ofstream ofs_ceres_log(output_path);

    if (feature.is_initialized_)
        return true;
    ceres::Problem problem;
    Eigen::Isometry3d cam0_tranformation{Eigen::Isometry3d::Identity()};
    cam0_tranformation.linear() = (map_state_set_[feature.observation_uv_.begin()->first].quat_.toRotationMatrix());
    cam0_tranformation.translation() = (map_state_set_[feature.observation_uv_.begin()->first].position_);
    double initial_position_world[3] = {1.0, 1.0, 10.0};
    if (!TriangulatePoint(feature, cam0_tranformation, initial_position_world))
        return false;
    double depth_initial = initial_position_world[2];
    double position_world[3] = {initial_position_world[0] / initial_position_world[2],
                                initial_position_world[1] / initial_position_world[2],
                                1.0 / initial_position_world[2]};

    for (auto &observation : feature.observation_uv_)
    {
        auto &camera_id = observation.first;
        const auto &camera_state = map_state_set_[camera_id];
        cv::Point2f &observation_point = observation.second;
        Eigen::Isometry3d cami_tranformation{Eigen::Isometry3d::Identity()};
        cami_tranformation.linear() = (camera_state.quat_.toRotationMatrix());
        cami_tranformation.translation() = (camera_state.position_);
        Eigen::Isometry3d cam0_cami = (cam0_tranformation.inverse() * cami_tranformation);
        ceres::CostFunction *cost_function = ReprojectionError::Create(observation_point, cam0_cami);
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), position_world);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);

    feature.position_world_ << position_world[0], position_world[1], 1.0;
    feature.position_world_ *= (1 / position_world[2]);

    double delta_depth = fabs(depth_initial - feature.position_world_(2)) / feature.position_world_(2);
    // ofs_ceres_log << std::fixed << std::setprecision(3) << feature.feature_id_ << "\t"
    //               << depth_initial << "\t"
    //               << feature.position_world_(2) << std::endl;

    if (feature_log_enable_)
    {
        ofs_feature_log_file_ << "Ceres Optizmation: \t" << feature.feature_id_ << "\t"
                              << feature.position_world_.transpose() << "\t"
                              << initial_position_world[2] << std::endl;
        for (auto iter = feature.raw_uv_.begin(); iter != feature.raw_uv_.end(); iter++)
        {
            const auto &camera_state_tmp = map_state_set_[iter->first];
            ofs_feature_log_file_ << "uv_feature_pixel: \t" << camera_state_tmp.state_id_ << "\t"
                                  << camera_state_tmp.time_.SecondOfWeek() << "\t"
                                  << feature.observation_uv_.at(iter->first) << "\t"
                                  << iter->second << std::endl;
        }
        if (delta_depth > 0.25 || summary.final_cost > 1e-3 || feature.position_world_(2) > 200 || feature.position_world_(2) < 1 || summary.num_successful_steps > 10)
        {
            ofs_feature_log_file_ << "do not used" << std::endl;
        }
        ofs_feature_log_file_ << std::endl
                              << std::endl;
    }

    if (delta_depth > 0.25 || summary.final_cost > 1e-3 || feature.position_world_(2) > 200 || feature.position_world_(2) < 1 || summary.num_successful_steps > 10)
    {
        // feature.position_world_ *= 0;
        return false;
    }
    // LOG(INFO) << "summary.final_cost: " << summary.final_cost << std::endl;
    //TODO  // feature.position_world_ = cam0_tranformation * feature.position_world_;

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
    static const bool cam_imu_rotation_eval = config_->get<int>("evaluate_camera_imu_rotation") != 0;
    Eigen::Matrix2d J_camera_mat;
    J_camera_mat << fx, 0.0,
        0.0, fy;

    int state_count = map_state_set_.size();
    int observe_feature_count = feature.observation_uv_.size();
    const utiltool::StateIndex &state_index = filter_->GetStateIndex();
    assert(state_count == state_index.camera_state_index.size());

    //* 一个相机状态观测到一个feature提供两个残差[u,v]
    H_state = Eigen::MatrixXd::Zero(observe_feature_count * 2, state_count * 6 + state_index.total_state);
    z_measure = Eigen::VectorXd::Zero(observe_feature_count * 2);
    Eigen::MatrixXd H_feature = Eigen::MatrixXd::Zero(observe_feature_count * 2, 3);

    int row_index = 0;
    auto &point_coor_world = feature.position_world_;
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << std::fixed << std::setprecision(4)
                                 << "time:  " << curr_time_.SecondOfWeek() << std::endl;
        ofs_msckf_info_log_file_ << "measure feature position: " << feature.feature_id_ << " "
                                 << feature.position_world_.transpose() << std::endl
                                 << "J_camera_mat: " << J_camera_mat.row(0) << "  "
                                 << J_camera_mat.row(1) << std::endl;
    }
    for (auto member : feature.observation_uv_)
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
        // Eigen::Vector2d point_f_pixel(point_f_c(0) / point_f_c(2) * fx + cx, point_f_c(1) / point_f_c(2) * fy + cy);
        Eigen::Vector2d point_f_pixel(point_f_c(0) / point_f_c(2), point_f_c(1) / point_f_c(2));

        if (msckf_info_log_enable_)
        {
            ofs_msckf_info_log_file_ << std::fixed << std::setprecision(8)
                                     << "l_cf_w: " << l_cf_w.transpose() << std::endl
                                     << "point_f_c: " << point_f_c.transpose() << std::endl
                                     << "point_f_pixel: " << point_f_pixel.transpose() << std::endl
                                     << "measure: " << measure << std::endl;
        }

        Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(2, 3);
        Eigen::MatrixXd J2_1 = Eigen::MatrixXd::Zero(3, 6);
        Eigen::MatrixXd J2_2 = Eigen::MatrixXd::Zero(3, 3);

        const auto &z = point_f_c(2);
        const auto &y = point_f_c(1);
        const auto &x = point_f_c(0);
        J1 << 1.0 / z, 0, -x / z / z,
            0, 1.0 / z, -y / z / z;

        // J1 = J_camera_mat * J1;
        J2_1.block<3, 3>(0, 3) = -1 * camera_state.quat_.toRotationMatrix().transpose() *
                                 utiltool::skew(l_cf_w); //* 相机姿态系数

        J2_1.block<3, 3>(0, 0) = -1 * camera_state.quat_.toRotationMatrix().transpose(); //* 相机位置系数

        J2_2 = camera_state.quat_.toRotationMatrix().transpose(); //* 特征点系数

        H_feature.block<2, 3>(row_index, 0) = J1 * J2_2;
        H_state.block<2, 6>(row_index, iter_camera_index->second) = J1 * J2_1;
        z_measure.segment<2>(row_index) << point_f_pixel(0) - measure.x, point_f_pixel(1) - measure.y;

        // auto gravity = earth::CalculateGravity(camera_state.position_);
        // LOG_FIRST_N(ERROR, 1) << gravity.transpose() << std::endl;

        // Eigen::Matrix<double, 2, 6> A = J1 * J2_1;

        // Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
        // u.block<3, 1>(0, 0) = utiltool::skew(l_cf_w) * gravity;
        // u.block<3, 1>(3, 0) = camera_state.quat_.toRotationMatrix().transpose() * gravity;

        // H_state.block<2, 6>(row_index, iter_camera_index->second) =
        //     (A - A * u * (u.transpose() * u).inverse() * u.transpose());

        // H_feature.block<2, 3>(row_index, 0) =
        //     -H_state.block<2, 3>(row_index, iter_camera_index->second);
        if (cam_imu_rotation_eval)
        {
            Eigen::MatrixXd J2_3 = utiltool::skew(camera_state.quat_.toRotationMatrix().transpose() * l_cf_w);
            H_state.block<2, 3>(row_index, state_index.camera_rotation_index_) = J1 * J2_3;
        }
        if (ofs_msckf_info_log_file_)
        {
            ofs_msckf_info_log_file_ << std::fixed << std::setprecision(10)
                                     << "camera_state_position: " << camera_state.position_.transpose() << std::endl
                                     << "camera_state_rotation: " << std::endl
                                     << camera_state.quat_.toRotationMatrix() << std::endl
                                     << "J1: " << std::endl
                                     << J1 << std::endl
                                     << "J2_1: " << std::endl
                                     << J2_1 << std::endl
                                     << "J2_2: " << std::endl
                                     << J2_2 << std::endl
                                     << "H_state-block:" << std::endl
                                     << H_state.block<2, 6>(row_index, iter_camera_index->second) << std::endl
                                     << "H_feature-block: " << std::endl
                                     << H_feature.block<2, 3>(row_index, 0) << std::endl
                                     << "z_measure: " << std::endl
                                     << z_measure << std::endl;
        }
        row_index += 2;
    }
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << "Hfeature:" << std::endl
                                 << std::setprecision(8) << H_feature << std::endl;
        ofs_msckf_info_log_file_ << "Hstate:" << std::endl
                                 << std::setprecision(8) << H_state << std::endl;
        ofs_msckf_info_log_file_ << "z_measure:" << std::endl
                                 << std::setprecision(8) << z_measure << std::endl;
    }
    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/measure_std.log");
    // static std::ofstream ofs_measure_log(output_path);
    double measure_mean = z_measure.mean();
    double measure_std = 0;
    for (size_t i = 0; i < z_measure.rows(); i++)
    {
        measure_std += (z_measure(i) - measure_mean) * (z_measure(i) - measure_mean) / (z_measure.rows() - 1);
        if (fabs(z_measure(i) * fx) > 4.0)
        {
            measure_std = 25.0 / fx / fx;
            break;
        }
    }
    measure_std = sqrt(measure_std);
    if (measure_std * fx > 2.0)
        return false;
    // ofs_measure_log << std::setw(10) << feature.feature_id_ << "   " << std::fixed << std::setprecision(5) << measure_std * fx << "  "
    //                 << measure_mean * fx << "  "
    //                 << (z_measure * fx).transpose() << std::endl;
    //* SVD求解Hf的左零空间，消除Hf
    Eigen::JacobiSVD<Eigen::MatrixXd>
        svd_helper(H_feature, Eigen::ComputeFullU | Eigen::ComputeThinV);
    Eigen::MatrixXd A = svd_helper.matrixU().rightCols(observe_feature_count * 2 - 3);
    H_state = A.transpose() * H_state;
    z_measure = A.transpose() * z_measure;
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << "AT:" << std::endl
                                 << std::setprecision(8) << A.transpose() << std::endl;
        ofs_msckf_info_log_file_ << "Hstate2:" << std::endl
                                 << std::setprecision(8) << H_state << std::endl;
        ofs_msckf_info_log_file_ << "z_measure2:" << std::endl
                                 << std::setprecision(8) << z_measure << std::endl;
        ofs_msckf_info_log_file_ << std::endl;
    }

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
    static double pixel_noise = config_->get<double>("camera_pixel_noise") / camera_mat_.at<double>(0, 0);

    Eigen::MatrixXd H_thin;
    Eigen::VectorXd Z_thin;
    if (false)//H_state.rows() > H_state.cols()) //TODO待更新为QR后进行量测
    {
        Eigen::HouseholderQR<Eigen::MatrixXd> qr;
        qr.compute(H_state);
        Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
        Eigen::MatrixXd Q = qr.householderQ();
        // ofs_feature_log_file_ << std::fixed << std::setprecision(12) << "R: " << std::endl
        //                              << R << std::endl;
        Eigen::VectorXd r_temp;
        r_temp = Q.transpose() * z_measure;
        // ofs_feature_log_file_
        //     << std::fixed << std::setprecision(12) << "H_state: " << std::endl
        //     << H_state << std::endl;
        // ofs_feature_log_file_ << std::fixed << std::setprecision(12) << "z_measure: " << std::endl
        //                              << z_measure << std::endl;
        // ofs_feature_log_file_ << std::fixed << std::setprecision(12) << "H_temp: " << std::endl
        //                              << H_temp << std::endl;
        // ofs_feature_log_file_ << std::fixed << std::setprecision(12) << "r_temp: " << std::endl
        //                              << r_temp << std::endl;
        // ofs_feature_log_file_.flush();
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
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << "H_thin:" << std::endl
                                 << std::fixed << std::setprecision(8) << H_thin << std::endl;
        ofs_msckf_info_log_file_ << "Z_thin:" << std::endl
                                 << std::setprecision(8) << Z_thin << std::endl;
        ofs_msckf_info_log_file_ << "measure_noise:" << pixel_noise * pixel_noise << std::endl;
    }
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
    static int debug_index_num = 0;
    int camera_state_count = 0;
    int camera_imu_rotation_idx = (config_->get<int>("evaluate_camera_imu_rotation") != 0) ? 3 : 0;

    assert(dx_camera.size() == (map_state_set_.size() * 6 + camera_imu_rotation_idx));

    if (camera_state_log_enable_)
    {
        ofs_camera_state_log_file_ << "before revise: " << map_state_set_ << std::endl;
    }
    if (camera_imu_rotation_idx == 3)
    {
        Eigen::Vector3d theta_camera = dx_camera.segment<3>(0);
        auto camera_quat_delta = RotationVector2Quaternion(theta_camera);
        cam_imu_tranformation_.rotate(camera_quat_delta.conjugate());
    }

    for (auto &camera_state : map_state_set_)
    {
        Eigen::Vector3d t_camera = dx_camera.segment<3>(camera_state_count * 6 + camera_imu_rotation_idx);
        Eigen::Vector3d theta_camera = dx_camera.segment<3>(camera_state_count * 6 + 3 + camera_imu_rotation_idx);
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

    if (camera_state_log_enable_)
    {
        ofs_camera_state_log_file_ << std::fixed << std::setprecision(8)
                                   << "dx_camera: " << dx_camera.transpose() << std::endl;
    }
    if (camera_state_log_enable_)
    {
        ofs_camera_state_log_file_ << "after revise: " << map_state_set_ << std::endl;
        ofs_camera_state_log_file_ << std::endl;
        auto att = utiltool::attitude::RotationMartix2Euler(cam_imu_tranformation_.rotation());
        ofs_camera_state_log_file_ << std::fixed << std::setprecision(8)
                                   << "cam_imu_rotation: " << att.transpose() * constant::rad2deg << std::endl;
        ofs_camera_state_log_file_ << std::endl;
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
            if (camera_state_log_enable_)
            {
                ofs_camera_state_log_file_ << std::fixed << std::setprecision(8)
                                           << "remove camera id: " << iter_camera->first
                                           << "\tindex_camera: " << (iter_index_camera->second) << std::endl;
            }
            // ofs_feature_log_file_ << std::fixed << std::setprecision(8) << "iter_camera->first: " << iter_camera->first
            //                              << "\t iter_index_camera: " << (iter_index_camera->second) << std::endl;
            // ofs_feature_log_file_ << "state_cov before: " << std::endl
            //                              << filter_->GetStateCov() << std::endl;
            filter_->EliminateIndex(iter_index_camera->second, 6);
            // ofs_feature_log_file_ << "state_cov after: " << std::endl
            //                              << filter_->GetStateCov() << std::endl;
            // ofs_feature_log_file_ << "index.camera_state_index before: " << index.camera_state_index << std::endl;
            iter_index_camera = index.camera_state_index.erase(iter_index_camera);
            for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
            {
                iter_index_camera->second -= 6;
            }
            iter_camera = map_state_set_.erase(iter_camera);

            if (camera_state_log_enable_)
            {
                ofs_camera_state_log_file_ << "index.camera_state_index after: "
                                           << index.camera_state_index << std::endl;
                ofs_camera_state_log_file_ << "map_state_set_ after: " << map_state_set_ << std::endl;
            }
            // ofs_feature_log_file_ << "index.camera_state_index after: " << index.camera_state_index << std::endl;
            // ofs_feature_log_file_ << "map_state_set_ before: " << map_state_set_ << std::endl;
            // ofs_feature_log_file_ << "map_state_set_ after: " << map_state_set_ << std::endl;
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
        bool temp = LMOptimizatePositionAndCheck(iter_feature->second);
        if (temp)
        {
            Eigen::MatrixXd H_state;
            Eigen::VectorXd z_measure;
            iter_feature->second.is_initialized_ = true;

            if (MeasurementJacobian(iter_feature->second, H_state, z_measure))
            {
                if (GatingTest(H_state, z_measure, (z_measure.rows() + 3) / 2))
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
                    if (msckf_info_log_enable_)
                    {
                        ofs_msckf_info_log_file_ << "GatingTest failed: "
                                                 << iter_feature->second.feature_id_ << "\t"
                                                 << iter_feature->second.position_world_.transpose() << std::endl;
                    }
                }
            }
        }
    }
    if (Hstate_all.rows() < 20 || count < 3)
        return;
    LOG(INFO) << std::fixed << std::setprecision(3) << curr_time_.SecondOfWeek() << " Measure Feature Number is " << count << std::endl;

    Eigen::VectorXd dx = MeasurementUpdate(Hstate_all, Zmeasure_all);
    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/res.log");
    // static std::ofstream ofs_dx_log(output_path);
    // Eigen::VectorXd v = Hstate_all * dx - Zmeasure_all;
    // ofs_dx_log << std::fixed << std::setprecision(3) << navinfo.time_.SecondOfWeek() << " "
    //            << std::fixed << std::setprecision(8) << v.transpose() * camera_mat_.at<double>(0, 0) << std::endl;

    if (dx.segment<3>(index.pos_index_).norm() > 1.0 || dx.segment<3>(index.vel_index_).norm() > 1.0)
    {
        LOG(ERROR) << "The value of resive dx is not correct, time: "
                   << std::setprecision(3) << navinfo.time_.SecondOfWeek()
                   << std::setprecision(8) << "\tpos: " << dx.segment<3>(index.pos_index_).transpose()
                   << "\tvel: " << dx.segment<3>(index.vel_index_).transpose() << std::endl;
        // return;
    }
    bool camera_imu_rotation = (config_->get<int>("evaluate_camera_imu_rotation") != 0);
    int revise_camimu_index = camera_imu_rotation ? 3 : 0;
    Eigen::VectorXd dx_imu = dx.head(filter_->GetStateIndex().total_state - revise_camimu_index);

    filter_->ReviseState(navinfo, dx_imu);
    ReviseCameraState(dx.tail(dx.size() - filter_->GetStateIndex().total_state + revise_camimu_index));
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << std::setprecision(3) << navinfo.time_.SecondOfWeek() << "  dx_imu: "
                                 << std::setprecision(6) << dx_imu.transpose() << std::endl;
    }
    navinfo.cam_imu_ = utiltool::attitude::RotationMartix2Euler(cam_imu_tranformation_.rotation());
    map_observation_set_.clear();
}

/**
 * @brief  查找移除的两个状态量,并更新对应的特征点
 * @note   
 * @retval None
 */
void MsckfProcess::RemoveRedundantCamStates()
{
    std::vector<StateId> rm_cam_state_ids;
    auto first_cam_state_iter = map_state_set_.begin();
    rm_cam_state_ids.emplace_back(first_cam_state_iter->first);
    ++first_cam_state_iter;
    rm_cam_state_ids.emplace_back(first_cam_state_iter->first);
    for (auto iter_member = map_feature_set_.begin(); iter_member != map_feature_set_.end();)
    {
        auto &feature = iter_member->second;
        auto iter_cam1 = feature.observation_uv_.find(rm_cam_state_ids[0]);
        auto iter_cam2 = feature.observation_uv_.find(rm_cam_state_ids[1]);
        if (iter_cam1 != feature.observation_uv_.end() || iter_cam2 != feature.observation_uv_.end())
        {
            if (CheckEnableTriangleate(feature) && CheckMotionStatus(feature) && feature.observation_uv_.size() > 5)
            {
                map_observation_set_.insert(*iter_member);
            }
            auto iter_feature_inmap = std::find_if(keypoints_featureid_.begin(), keypoints_featureid_.end(),
                                                   [&](std::pair<unsigned long long int, long long int> feature_iter) {
                                                       return feature_iter.second == iter_member->first;
                                                   });
            if (iter_feature_inmap == keypoints_featureid_.end())
            {
                LOG(FATAL) << "keypoint_id find error" << std::endl;
            }
            keypoints_featureid_.erase(iter_feature_inmap);
            for (auto &iter : map_state_set_)
            {
                auto &feature_id_vector = iter.second.feature_id_set_;
                std::remove(std::begin(feature_id_vector), std::end(feature_id_vector), iter_member->first);
            }
            iter_member = map_feature_set_.erase(iter_member);
            continue;
        }
        ++iter_member;
    }
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

    // for (int i = 0; i < 2; ++i)
    // {
    //     const Eigen::Vector3d position = cam_state_iter->second.position_;
    //     const Eigen::Matrix3d rotation = cam_state_iter->second.quat_.toRotationMatrix();

    //     double distance = (position - key_position).norm();
    //     double angle = Eigen::AngleAxisd(rotation * key_rotation.transpose()).angle();

    //     if (angle < 0.2618 && distance < 0.4 /*&& tracking_rate_ > 0.5*/)
    //     {
    //         rm_cam_state_ids.push_back(cam_state_iter->first);
    //         ++cam_state_iter;
    //     }
    //     else
    //     {
    //         rm_cam_state_ids.push_back(first_cam_state_iter->first);
    //         ++first_cam_state_iter;
    //     }
    // }
    // std::sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());
    rm_cam_state_ids.emplace_back(first_cam_state_iter->first);
    ++first_cam_state_iter;
    rm_cam_state_ids.emplace_back(first_cam_state_iter->first);
    if (msckf_info_log_enable_)
    {
        ofs_msckf_info_log_file_ << "rm_cam_ids: ";
        for (auto &iter : rm_cam_state_ids)
        {
            ofs_msckf_info_log_file_ << iter << "\t";
        }
        ofs_msckf_info_log_file_ << std::endl;
    }

    map_observation_set_.clear();
    for (auto iter_member = map_feature_set_.begin(); iter_member != map_feature_set_.end();)
    {
        auto &feature = iter_member->second;
        auto iter_cam1 = feature.observation_uv_.find(rm_cam_state_ids[0]);
        auto iter_cam2 = feature.observation_uv_.find(rm_cam_state_ids[1]);
        if (iter_cam1 != feature.observation_uv_.end() && iter_cam2 != feature.observation_uv_.end())
        {
            if (CheckEnableTriangleate(feature) && CheckMotionStatus(feature))
            {
                map_observation_set_.insert(*iter_member);
            }
            auto &key_id = iter_member->first;
            auto iter_feature_inmap = std::find_if(keypoints_featureid_.begin(), keypoints_featureid_.end(),
                                                   [&](std::pair<unsigned long long int, long long int> feature_iter) {
                                                       return feature_iter.second == iter_member->first;
                                                   });
            if (iter_feature_inmap == keypoints_featureid_.end())
            {
                LOG(FATAL) << "keypoint_id find error" << std::endl;
            }
            keypoints_featureid_.erase(iter_feature_inmap);
            for (auto &iter : map_state_set_)
            {
                auto &feature_id_vector = iter.second.feature_id_set_;
                std::remove(std::begin(feature_id_vector), std::end(feature_id_vector), iter_member->first);
            }
            iter_member = map_feature_set_.erase(iter_member);
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
        ++iter_member;
    }

    //* 更新状态
    FeatureMeasureUpdate(navinfo);

    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/elimibate_state.log");
    // static std::ofstream ofs_feature_log_file_(output_path);
    // //** 移除这两个相机状态
    // ofs_feature_log_file_ << std::fixed <<std::setprecision(8)<< curr_time_.SecondOfWeek() <<" remove camera_state"<< std::endl;
    auto &index = filter_->GetStateIndex();
    for (auto iter_camera : rm_cam_state_ids)
    {
        /* code */
        auto iter_index_camera = index.camera_state_index.find(iter_camera);
        assert(iter_index_camera != index.camera_state_index.end());
        // ofs_feature_log_file_ << "iter_camera->first: (camera_remove)" << iter_camera
        //                              << "\t iter_index_camera: " << (iter_index_camera->second) << std::endl;
        // ofs_feature_log_file_ << "state_cov before: " << std::endl
        //                              << filter_->GetStateCov() << std::endl;
        filter_->EliminateIndex(iter_index_camera->second, 6);
        // ofs_feature_log_file_ << "state_cov after: " << std::endl
        //                              << filter_->GetStateCov() << std::endl;
        // ofs_feature_log_file_ << "index.camera_state_index before: " << index.camera_state_index << std::endl;

        iter_index_camera = index.camera_state_index.erase(iter_index_camera);
        for (; iter_index_camera != index.camera_state_index.end(); iter_index_camera++)
        {
            iter_index_camera->second -= 6;
        }
        // ofs_feature_log_file_ << "index.camera_state_index after: " << index.camera_state_index << std::endl;
        // ofs_feature_log_file_ << "map_state_set_ before: " << map_state_set_ << std::endl;
        map_state_set_.erase(iter_camera);
        // ofs_feature_log_file_ << "map_state_set_ after: " << map_state_set_ << std::endl;
        // ofs_feature_log_file_ << std::endl;
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
    static double tmp_pixel_noise = 5.0 / camera_mat_.at<double>(0, 0);
    static double threshold = 20 / camera_mat_.at<double>(0, 0);
    const auto &state_cov = filter_->GetStateCov();
    Eigen::MatrixXd P1 = H * state_cov * H.transpose();
    Eigen::MatrixXd P2 = tmp_pixel_noise * tmp_pixel_noise * Eigen::MatrixXd::Identity(H.rows(), H.rows());
    double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);
    double average = 0;
    for (int i = 0; i < r.rows(); ++i)
    {
        average += fabs(r(i)) / r.rows();
        if (fabs(r(i)) > threshold)
        {
            average = threshold;
            break;
        }
    }
    if (gamma < utiltool::chi_squared_test_table[dof - 1]) //&& average < threshold / 2)
    {
        return true;
    }
    else
    {
        LOG(INFO) << std::fixed << std::setprecision(3) << curr_time_.SecondOfWeek() << "\tgamma: " << gamma << "\tdof:" << dof << "\t average" << average << std::endl;
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

bool MsckfProcess::CheckStaticMotion()
{
    double mean_motion = 0.0;
    int size = pre_frame_keypoints_.size();
    for (int i = 0; i < size; ++i)
    {
        double dx = curr_frame_keypoints_[i].x - pre_frame_keypoints_[i].x;
        double dy = curr_frame_keypoints_[i].y - pre_frame_keypoints_[i].y;
        mean_motion += sqrt(dx * dx + dy * dy) / size;
    }
    if (mean_motion < 1.2)
    {
        return true;
    }
    return false;
}

void MsckfProcess::Test()
{
    static std::string test_file_path = config_->get<std::string>("result_output_path") + "/test.log";
    static std::ofstream ofs_test_file(test_file_path);

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
    ofs_test_file << "camera_imu:" << curr_time_.SecondOfWeek() << " " << curr_camera_state->second.time_.SecondOfWeek() << " " << std::fixed
                  << std::setprecision(8) << rotation_imu.transpose() * constant::rad2deg << " "
                  << rotation_cam.transpose() * constant::rad2deg << " "
                  << Rimu12.transpose() * constant::rad2deg << " "
                  << t_imu.transpose() * 100 << "  "
                  << t_camera.transpose() * 100 << "  "
                  << std::endl;
}

bool MsckfProcess::LMOptimizatePositionAndCheck(Feature &feature)
{
    // static std::string output_path = (config_->get<std::string>("result_output_path")) + ("/ceres_lm_out.log");
    // static std::ofstream ofs_ceres2_log(output_path);

    if (feature.observation_uv_.size() < 5)
    {
        return false;
    }
    Eigen::Isometry3d cam0_t{Eigen::Isometry3d::Identity()};
    cam0_t.linear() = (map_state_set_[feature.observation_uv_.begin()->first].quat_.toRotationMatrix());
    cam0_t.translation() = (map_state_set_[feature.observation_uv_.begin()->first].position_);

    Feature all_feature(feature.feature_id_);
    Feature part1_feature(feature.feature_id_);
    Feature part2_feature(feature.feature_id_);
    size_t i = 0;
    auto iter = feature.observation_uv_.begin();
    for (; iter != feature.observation_uv_.end(); iter++)
    {
        all_feature.observation_uv_[iter->first] = iter->second;
        if (i % 3 == 0)
        {
            part1_feature.observation_uv_[iter->first] = iter->second;
            part2_feature.observation_uv_[iter->first] = iter->second;
        }
        else if (i % 3 == 1)
        {
            part1_feature.observation_uv_[iter->first] = iter->second;
        }
        else
        {
            part2_feature.observation_uv_[iter->first] = iter->second;
        }
        i++;
    }
    if (LMOptimizatePosition(all_feature) &&
        LMOptimizatePosition(part1_feature) &&
        LMOptimizatePosition(part2_feature))
    {
        Eigen::Vector3d diff1 = all_feature.position_world_ - part1_feature.position_world_;
        Eigen::Vector3d diff2 = all_feature.position_world_ - part2_feature.position_world_;
        // ofs_ceres2_log << std::fixed << std::setprecision(3) << all_feature.feature_id_ << "\t"
        //                << std::setw(12) << all_feature.position_world_.transpose() << " "
        //                << std::setw(12) << part1_feature.position_world_.transpose() << " "
        //                << std::setw(12) << part2_feature.position_world_.transpose() << " " << std::endl;
        if (abs(diff1(2)) < 8 && abs(diff2(2)) < 8 &&
            abs(diff1(1)) < 2 && abs(diff2(1)) < 2 &&
            abs(diff1(0)) < 2 && abs(diff2(0)) < 2)
        {
            feature.position_world_ = cam0_t * all_feature.position_world_;
            feature.is_initialized_ = true;
            return true;
        }
    }
    return false;
}

} // namespace camera

} // namespace mscnav