/*
** navinitialized.cc for mscnav in /media/fwt/Data/program/mscnav/src/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午9:14:23 2019 little fang
** Last update Wed Mar 10 上午11:51:53 2020 little fang
*/
#include "imu/navinitialized.h"
#include "navearth.hpp"
#include "navattitude.hpp"
#include "constant.hpp"
#include <algorithm>
#include <numeric>

using namespace utiltool;
using namespace attitude;
using namespace constant;
using Eigen::Vector3d;

namespace mscnav
{
namespace
{
GnssData::Ptr &InterpolateGnssVel(const GnssData::Ptr &first, GnssData::Ptr &second)
{
    second->vel_ = (second->pos_ - first->pos_) / (second->get_time() - first->get_time());
    return second;
}

NavInfo &AttCaln2e(NavInfo &nav_info)
{
    Vector3d blh = earth::WGS84XYZ2BLH(nav_info.pos_);
    nav_info.rotation_ = earth::CalCn2e(blh(0), blh(1)) * nav_info.rotation_;
    nav_info.quat_ = RotationMartix2Quaternion(nav_info.rotation_);
    return nav_info;
}

NavInfo &MotionAligned(const GnssData::Ptr &gnss_data, NavInfo &nav_info)
{
    nav_info.pos_ = gnss_data->pos_;
    nav_info.vel_ = gnss_data->vel_;
    Vector3d blh = earth::WGS84XYZ2BLH(nav_info.pos_);
    Vector3d vel_ned = earth::CalCe2n(blh(0), blh(1)) * nav_info.vel_;
    nav_info.att_(0) = 0.0;
    nav_info.att_(1) = atan2(-vel_ned(2), sqrt(pow(vel_ned(0), 2) + pow(vel_ned(1), 2)));
    nav_info.att_(2) = atan2(vel_ned(1), vel_ned(0));
    nav_info.rotation_ = Euler2RotationMatrix(nav_info.att_);
    nav_info.quat_ = RotationMartix2Quaternion(nav_info.rotation_);
    nav_info.time_ = gnss_data->get_time();
    nav_info.pos_std_ = gnss_data->pos_std_;
    nav_info.vel_std_ = gnss_data->vel_std_;
    return nav_info;
}
Euler AcceLeveling(std::vector<ImuData::Ptr> &imu_buffer)
{
    using constant::constant_g0;
    ImuData::Ptr imu = imu_buffer.front();
    auto start_time = imu->get_time();
    imu = std::accumulate(imu_buffer.begin() + 1, imu_buffer.end(), imu, imu_accumulate());
    imu->acce_ /= (imu->get_time() - start_time);
    imu->gyro_ /= (imu->get_time() - start_time);
    Euler euler(0, 0, 0);
    euler(0) = asin(imu->acce_(1) / constant_g0) * (imu->acce_[2] > 0 ? 1 : -1);
    euler(1) = asin(imu->acce_(0) / constant_g0) * (imu->acce_[2] > 0 ? -1 : 1);
    return euler;
}
} // namespace
/**
 * @brief  开启对准
 * @note   
 * @param  &nav_info:对准结束后的导航信息数据 
 * @retval 
 */
bool InitializedNav::StartAligning(utiltool::NavInfo &nav_info)
{
    GnssData::Ptr gnss_data;
    ImuData::Ptr imu_data;
    BaseData::bPtr data;
    std::vector<ImuData::Ptr> imu_data_buffer;
    static double velocity_threshold = config->get<double>("alignnment_velocity_threshold");
    bool use_set_attitude = (config->get<int>("alignnment_attitude_mode") != 0);
    bool use_set_posvel = (config->get<int>("alignnment_posvel_mode") != 0);
    /* 赋值其他设定项 */
    std::vector<double>
        vec_value_tmp = config->get_array<double>("leverarm");
    nav_info.leverarm_ << vec_value_tmp.at(0), vec_value_tmp.at(1), vec_value_tmp.at(2);

    vec_value_tmp = config->get_array<double>("initial_gyro_bias");
    nav_info.gyro_bias_ << vec_value_tmp.at(0) * dh2rs, vec_value_tmp.at(1) * dh2rs, vec_value_tmp.at(2) * dh2rs;

    vec_value_tmp = config->get_array<double>("initial_acce_bias");
    nav_info.acce_bias_
        << vec_value_tmp.at(0) * constant_mGal,
        vec_value_tmp.at(1) * constant_mGal,
        vec_value_tmp.at(2) * constant_mGal;

    vec_value_tmp = config->get_array<double>("initial_gyro_scale");
    nav_info.gyro_scale_
        << vec_value_tmp.at(0) * constant_ppm,
        vec_value_tmp.at(1) * constant_ppm,
        vec_value_tmp.at(2) * constant_ppm;

    vec_value_tmp = config->get_array<double>("initial_acce_scale");
    nav_info.acce_scale_
        << vec_value_tmp.at(0) * constant_ppm,
        vec_value_tmp.at(1) * constant_ppm,
        vec_value_tmp.at(2) * constant_ppm;

    data = ptr_data_queue_->GetData();
    if (data->get_type() == DATAUNKOWN)
    {
        LOG(ERROR) << "data stream error " << std::endl;
        return false;
    }
    while (data->get_type() != GNSSDATA)
    {
        data = ptr_data_queue_->GetData();
        LOG_EVERY_N(INFO, 100) << "Aligning!!! Waiting for GNSS data ... " << std::endl;
    }
    gnss_data = std::dynamic_pointer_cast<GnssData>(data);
    if (gnss_data->format_type_ == GNSSPOS)
    {
        data = ptr_data_queue_->GetData();
        while (data->get_type() != GNSSDATA)
        {
            data = ptr_data_queue_->GetData();
            LOG_EVERY_N(INFO, 100) << "Waiting GNSS data for differing velocity ... " << std::endl;
        }
        GnssData::Ptr temp_gnss_data = std::dynamic_pointer_cast<GnssData>(data);
        gnss_data = InterpolateGnssVel(gnss_data, temp_gnss_data);
    }
    if (use_set_attitude)
    {
        aligned_mode_ = SETTING;
        auto att = config->get_array<double>("initial_att");
        nav_info.pos_ = gnss_data->pos_;
        nav_info.vel_ = gnss_data->vel_;
        nav_info.att_(0) = att[0] * constant::deg2rad;
        nav_info.att_(1) = att[1] * constant::deg2rad;
        nav_info.att_(2) = att[2] * constant::deg2rad;
        nav_info.rotation_ = Euler2RotationMatrix(nav_info.att_);
        nav_info.quat_ = RotationMartix2Quaternion(nav_info.rotation_);
        nav_info.time_ = gnss_data->get_time();
        nav_info.pos_std_ = gnss_data->pos_std_;
        nav_info.vel_std_ = gnss_data->vel_std_;
        nav_info = AttCaln2e(nav_info);
        LOG(INFO) << "Alignment by setted value, " << nav_info.att_.transpose() * constant::rad2deg << std::endl;
        return true;
    }
    if (gnss_data->vel_.norm() > velocity_threshold)
    {
        aligned_mode_ = IN_MOTION;
        nav_info = MotionAligned(gnss_data, nav_info);
        nav_info = AttCaln2e(nav_info);
        return true; //面向车载设备,且设备坐标系与车体系差异不显著
    }
    else
    {
        aligned_mode_ = STATIONARY_AND_MOTION;
        int data_rate = config->get<int>("data_rate");
        while (true)
        {
            data = ptr_data_queue_->GetData();
            if (data->get_type() == IMUDATA)
            {
                imu_data_buffer.emplace_back(std::dynamic_pointer_cast<ImuData>(data));
                if (imu_data_buffer.size() > data_rate * 10)
                {
                    imu_data_buffer.erase(imu_data_buffer.begin());
                }
            }
            else if (data->get_type() == GNSSDATA)
            {
                if (gnss_data->format_type_ == GNSSPOS)
                {
                    GnssData::Ptr temp_gnss_data = std::dynamic_pointer_cast<GnssData>(data);
                    gnss_data = InterpolateGnssVel(gnss_data, temp_gnss_data);
                }
                if (gnss_data->vel_.norm() > velocity_threshold)
                {
                    nav_info = MotionAligned(gnss_data, nav_info);
                    Euler euler = AcceLeveling(imu_data_buffer);
                    euler(2) = nav_info.att_(2);
                    nav_info.quat_ = Euler2Quaternion(euler);
                    nav_info.rotation_ = Quaternion2RotationMatrix(nav_info.quat_);
                    nav_info.att_ = euler;
                    nav_info = AttCaln2e(nav_info);
                    return true; //面向车载设备,且设备坐标系与车体系差异不显著
                }
            }
            else if (data->get_type() == DATAUNKOWN)
            {
                LOG(FATAL) << "初始对准失败" << std::endl;
            }
        }
    }
    return false;
}

/**
 * @brief  set the initialized variance of state
 * @note   
 * @param  &PVariance: 
 * @param  &nav_info: 
 * @retval 
 */
Eigen::VectorXd &InitializedNav::SetInitialVariance(Eigen::VectorXd &PVariance,
                                                    utiltool::NavInfo &nav_info,
                                                    const StateIndex &index)
{
    if (!index.is_initialized)
    {
        LOG(ERROR) << "state index do not determined. Please make sure it is intialized before" << std::endl;
        return PVariance;
    }
    PVariance = Eigen::VectorXd::Zero(index.total_state);
    bool user_define_std_pos_vel = config->get<int>("use_define_variance_pos_vel") == 0 ? false : true;
    if (user_define_std_pos_vel)
    {
        auto pos_std = config->get_array<double>("initial_pos_std");
        auto vel_std = config->get_array<double>("initial_vel_std");
        PVariance.segment<6>(index.pos_index_) << pos_std.at(0), pos_std.at(1), pos_std.at(2), vel_std.at(0), vel_std.at(1), vel_std.at(2);
    }
    else
    {
        PVariance.segment<3>(index.pos_index_) = nav_info.pos_std_;
        PVariance.segment<3>(index.vel_index_) = nav_info.vel_std_;
    }
    bool user_define_std_att = config->get<int>("use_define_variance_att") == 0 ? false : true;
    if (user_define_std_att || aligned_mode_ == SETTING)
    {
        auto att_std = config->get_array<double>("initial_att_std");
        PVariance.segment<3>(index.att_index_) << att_std.at(0) * deg2rad, att_std.at(1) * deg2rad, att_std.at(2) * deg2rad;
        LOG(INFO) << "The attitude initialized variance setted by configure file!!! configure file!!!" << std::endl;
    }
    else
    {
        if (aligned_mode_ == IN_MOTION)
            PVariance.segment<3>(index.att_index_) << 5.0_deg, 5.0_deg, 5.0_deg;
        if (aligned_mode_ == STATIONARY_AND_MOTION)
            PVariance.segment<3>(index.att_index_) << 2.0_deg, 2.0_deg, 5.0_deg;
        LOG(INFO) << "aligned_mode is " << aligned_mode_ << std::endl;
        LOG(INFO) << "The attitude initialized variance setted by default!!! default!!!" << std::endl;
    }
    auto gyro_bias_std = config->get_array<double>("gyro_bias_std");
    auto acce_bias_std = config->get_array<double>("acce_bias_std");
    PVariance.segment<3>(index.gyro_bias_index_) << gyro_bias_std.at(0), gyro_bias_std.at(1), gyro_bias_std.at(2);
    PVariance.segment<3>(index.gyro_bias_index_) *= constant::dh2rs ;
    PVariance.segment<3>(index.acce_bias_index_) << acce_bias_std.at(0), acce_bias_std.at(1), acce_bias_std.at(2);
    PVariance.segment<3>(index.acce_bias_index_) *= constant_mGal ;
    bool evaluate_imu_scale = config->get<int>("evaluate_imu_scale") == 0 ? false : true;
    if (evaluate_imu_scale)
    {
        auto gyro_scale_std = config->get_array<double>("gyro_scale_std");
        auto acce_scale_std = config->get_array<double>("acce_scale_std");
        PVariance.segment<3>(index.gyro_scale_index_) << gyro_scale_std.at(0), gyro_scale_std.at(1), gyro_scale_std.at(2);
        PVariance.segment<3>(index.gyro_scale_index_) *= constant_ppm ;
        PVariance.segment<3>(index.acce_scale_index_) << acce_scale_std.at(0), acce_scale_std.at(1), acce_scale_std.at(2);
        PVariance.segment<3>(index.acce_scale_index_) *= constant_ppm ;
    }
    bool evaluate_camera_imu_rotation = config->get<int>("evaluate_camera_imu_rotation") != 0 ? true : false;
    if (evaluate_camera_imu_rotation)
    {
        PVariance.segment<3>(index.camera_rotation_index_) << 0.5_deg, 0.5_deg, 0.5_deg;
    }
    //TODO 使用其他状态量是确定其初始方差.
    return PVariance;
}
/**
 * @brief  Determine the index of the estmiate state
 * @note   
 * @param  &state_index: 
 * @retval None
 */
void InitializedNav::SetStateIndex(utiltool::StateIndex &state_index)
{
    int evaluate_imu_scale = config->get<int>("evaluate_imu_scale");
    bool evaluate_camera_imu_rotation = config->get<int>("evaluate_camera_imu_rotation") != 0 ? true : false;
    // bool rotation_iv = config->get<int>("evaluate_imu_vehicle_rotation");
    state_index.pos_index_ = 0;
    state_index.vel_index_ = 3;
    state_index.att_index_ = 6;
    state_index.gyro_bias_index_ = 9;
    state_index.acce_bias_index_ = 12;
    int basic_index = 12;
    if (evaluate_imu_scale != 0)
    {
        state_index.gyro_scale_index_ = basic_index + 3;
        state_index.acce_scale_index_ = basic_index + 6;
        basic_index += 6;
    }
    if (evaluate_camera_imu_rotation)
    {
        state_index.camera_rotation_index_ = basic_index + 3;
        basic_index += 3;
    }
    state_index.total_state = basic_index + 3; //TODO 需要调整
    // TODO 使用里程计,相机等需要更新其他状态量的参数索引
    // state_index.odometer_scale_index;
    // state_index.imu_vehicle_rotation_index_;
    // state_index.camera_delay_index_;
    // state_index.camera_rotation_index_;
    // state_index.camera_translation_index_;
    state_index.is_initialized = true;
}
} // namespace mscnav
