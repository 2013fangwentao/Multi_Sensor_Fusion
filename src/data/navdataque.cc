/*
** navdataque.cc for mscnav in /media/fwt/Data/program/mscnav/src/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午9:49:00 2019 little fang
** Last update Wed Jul 23 上午11:09:46 2019 little fang
*/
#include "data/navdataque.h"
#include "navlog.hpp"

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
    GnssData::Ptr gnss = std::make_shared<GnssData>();
    ImuData::Ptr imu = std::make_shared<ImuData>();
    BaseData::bPtr data = std::make_shared<BaseData>();
    int data_type_count = 2; //TODO 多源数据时需要重新考虑赋值问题.动态变化
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
        data = TimeFirst(bptr_gnss, bptr_imu);
        if (data != nullptr)
        {
            mtx_data_.lock();
            data_queue_.emplace_back(data);
            mtx_data_.unlock();
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
                if (!ptr_imu_data_->GetImuData(imu))
                {
                    imu = nullptr;
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
