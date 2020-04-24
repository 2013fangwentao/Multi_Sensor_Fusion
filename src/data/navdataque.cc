/*
** navdataque.cc for mscnav in /media/fwt/Data/program/mscnav/src/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午9:49:00 2019 little fang
** Last update Sun Nov 9 上午9:08:30 2019 little fang
*/
#include "data/navdataque.h"
#include "navlog.hpp"
#include "navconfig.hpp"

using namespace utiltool;

namespace mscnav
{

utiltool::BaseData::bPtr DataQueue::GetData()
{
    utiltool::BaseData::bPtr data = std::make_shared<BaseData>();
    if (data_queue_.size() > 0)
    {
        std::unique_lock<std::mutex> lck(mtx_data_);
        data = data_queue_.front();
        data_queue_.pop_front();
        return data;
    }
    else if (aint_mark_data_flags_ == 0)
    {
        while (data_queue_.size() == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            LOG_EVERY_N(INFO, 200) << "BaseData thread sleep 10s" << std::endl;
            if (aint_mark_data_flags_ == 1)
            {
                return data;
            }
        }
        std::unique_lock<std::mutex> lck(mtx_data_);
        data = data_queue_.front();
        data_queue_.pop_front();
        return data;
    }
    else
    {
        return data;
    }
}

void DataQueue::SortData()
{
    using namespace camera;
    ConfigInfo::Ptr config = ConfigInfo::GetInstance();

    GnssData::Ptr gnss = std::make_shared<GnssData>();
    ImuData::Ptr imu = std::make_shared<ImuData>();
    ImuData::Ptr imu_bak = std::make_shared<ImuData>();
    BaseData::bPtr data = std::make_shared<BaseData>();
    CameraData::Ptr camera = nullptr;
    int camera_enable = config->get<int>("camera_enable");
    int data_type_count = 2; //TODO 多源数据时需要重新考虑赋值问题.动态变化
    if (camera_enable != 0)
    {
        camera = std::make_shared<CameraData>();
        data_type_count++;
        if (ptr_camera_data_ == nullptr)
        {
            LOG(FATAL) << "Camera data thread error" << std::endl;
        }
        if (!ptr_camera_data_->GetData(camera))
        {
            LOG(FATAL) << "Camera Data Thread Over, Can not Get Data" << std::endl;
            return;
        }
    }
    if (!ptr_gnss_data_->GetData(gnss))
    {
        LOG(FATAL) << "Gnss Data Thread Over, Can not Get Data" << std::endl;
        return;
    }
    if (!ptr_imu_data_->GetImuData(imu))
    {
        LOG(FATAL) << "Imu Data Thread Over, Can not Get Data" << std::endl;
        return;
    }

    while (data_type_count > 0)
    {
        BaseData::bPtr bptr_gnss = (gnss);
        BaseData::bPtr bptr_imu = (imu);
        BaseData::bPtr bptr_camera = (camera);
        data = TimeFirst(bptr_gnss, bptr_imu, bptr_camera);
        if (data != nullptr)
        {
            if (data->get_type() != IMUDATA)
            {
                const auto &other_time = data->get_time();
                const auto &imu_time = imu->get_time();
                const auto &imu_bak_time = imu_bak->get_time();
                mtx_data_.lock();
                if (fabs(imu_bak_time - other_time) > 2e-3)
                {
                    const double dt = (other_time - imu_bak_time) / (imu_time - imu_bak_time);
                    ImuData::Ptr middle_imu = std::make_shared<ImuData>(other_time);
                    middle_imu->acce_ = imu->acce_ * dt;
                    middle_imu->gyro_ = imu->gyro_ * dt;
                    imu->acce_ -= middle_imu->acce_;
                    imu->gyro_ -= middle_imu->gyro_;
                    imu_bak = middle_imu;
                    BaseData::bPtr middle_imu_bptr = middle_imu;
                    data_queue_.emplace_back(middle_imu_bptr);
                    data_queue_.emplace_back(data);
                }
                else
                {
                    data_queue_.emplace_back(data);
                }
                mtx_data_.unlock();
            }
            else
            {
                mtx_data_.lock();
                data_queue_.emplace_back(data);
                mtx_data_.unlock();
            }

            switch (data->get_type())
            {
            case GNSSDATA:
                if (!ptr_gnss_data_->GetData(gnss))
                {
                    gnss = nullptr;
                    data_type_count--;
                }
                break;
            case IMUDATA:
                imu_bak = imu;
                if (!ptr_imu_data_->GetImuData(imu))
                {
                    imu = nullptr;
                    data_type_count--;
                }
                break;
            case CAMERADATA:
                if (!ptr_camera_data_->GetData(camera))
                {
                    camera = nullptr;
                    data_type_count--;
                }
                break;
            default: //bak for other data
                break;
            }
        }
        else
        {
            break;
        }

        while (data_queue_.size() > MAX_SIZE_IMUPOOL * 200)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    aint_mark_data_flags_ = 1;
}
} // namespace mscnav
