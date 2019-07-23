/*
** navinitialized.cc for mscnav in /media/fwt/Data/program/mscnav/src/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午9:14:23 2019 little fang
** Last update undefined Jul 21 下午9:14:23 2019 little fang
*/
#include "imu/navinitialized.h"
#include "navearth.hpp"
#include "navattitude.hpp"
using namespace utiltool;
using namespace attitude;
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
NavInfo &MotionAligned(const GnssData::Ptr &gnss_data, NavInfo &nav_info)
{
    nav_info.pos_ = gnss_data->pos_;
    nav_info.vel_ = gnss_data->vel_;
    Vector3d blh = earth::WGS84XYZ2BLH(nav_info.pos_);
    Vector3d vel_ned = earth::CalCe2n(blh(0), blh(1)) * nav_info.vel_;
    nav_info.att_(0) = 0.0;
    nav_info.att_(1) = -atan(vel_ned(2) / sqrt(pow(vel_ned(0), 2) + pow(vel_ned(1), 2)));
    nav_info.att_(2) = (atan(vel_ned(1) / vel_ned(0)));
    nav_info.quat_ = Euler2Quaternion(nav_info.att_);
    nav_info.time_ = gnss_data->get_time();
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
bool InitialiedNav::StartAligning(utiltool::NavInfo &nav_info)
{
    GnssData::Ptr gnss_data;
    ImuData::Ptr imu_data;
    BaseData::bPtr data;
    std::vector<ImuData::Ptr> imu_data_buffer;
    double velocity_threshold = config->get<double>("alignnment_velocity_threshold");
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
    if (gnss_data->vel_.norm() > velocity_threshold)
    {
        aligned_mode_ = IN_MOTION;
        nav_info = MotionAligned(gnss_data, nav_info);
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
                    nav_info.att_ = euler;
                    return true; //面向车载设备,且设备坐标系与车体系差异不显著
                }
            }
        }
    }
}

} // namespace mscnav
