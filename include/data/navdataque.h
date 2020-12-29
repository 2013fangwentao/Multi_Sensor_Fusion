/*
** navdataque.h for mscnav in /media/fwt/Data/program/mscnav/include/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午9:38:06 2019 little fang
** Last update Tue Aug 26 上午9:20:13 2019 little fang
*/

#ifndef DATA_DATAQUE_H_
#define DATA_DATAQUE_H_

#include "navgnss.h"
#include "navimu.h"
#include "navcamera.h"

namespace mscnav
{
class DataQueue
{
public:
    DataQueue(const GnssDataCollect::Ptr &ptr_gnss_data,
              const ImuDataCollect::Ptr &ptr_imu_data,
              const CameraDataCollect::Ptr &ptr_camera_data = nullptr)
        : ptr_gnss_data_(ptr_gnss_data),
          ptr_imu_data_(ptr_imu_data),
          ptr_camera_data_(ptr_camera_data)
    {
        aint_mark_data_flags_ = 0;
        std::thread th(&DataQueue::SortData, this);
        th_sort_data_ = std::move(th);
    }
    ~DataQueue()
    {
        if (th_sort_data_.joinable())
        {
            th_sort_data_.join();
        }
    }
    using Ptr = std::shared_ptr<DataQueue>;

public:
    utiltool::BaseData::bPtr GetData();

private:
    void SortData();

private:
    GnssDataCollect::Ptr ptr_gnss_data_;
    ImuDataCollect::Ptr ptr_imu_data_;
    CameraDataCollect::Ptr ptr_camera_data_;
    std::thread th_sort_data_;
    std::mutex mtx_data_;
    std::deque<utiltool::BaseData::bPtr> data_queue_;
    std::atomic<int> aint_mark_data_flags_{-1};
};

namespace
{
inline utiltool::BaseData::bPtr &TimeFirst(utiltool::BaseData::bPtr &t1, utiltool::BaseData::bPtr &t2)
{
    if (t1 == nullptr && t2 == nullptr)
    {
        return t1;
    }
    else if (t1 == nullptr)
    {
        return t2;
    }
    else if (t2 == nullptr)
    {
        return t1;
    }
    if (t1->get_time() < t2->get_time())
    {
        return t1;
    }
    else if (t1->get_time() == t2->get_time() && t1->get_type() == utiltool::IMUDATA)
    {
        return t1;
    }
    else
    {
        return t2;
    }
}
template <typename... Args>
inline utiltool::BaseData::bPtr &TimeFirst(utiltool::BaseData::bPtr &t, Args&... args)
{
    utiltool::BaseData::bPtr &t1 = (TimeFirst(args...));
    return TimeFirst(t, t1);
}

} // namespace
} // namespace mscnav

#endif /* !DATA_DATAQUE_H_ */
