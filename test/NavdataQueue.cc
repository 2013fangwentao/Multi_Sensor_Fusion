/*
** NavUnionTest.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Jul 22 下午12:26:50 2019 little fang
** Last update Wed Jul 23 上午10:52:05 2019 little fang
*/

#include "data/navdataque.h"
#include "navconfig.hpp"
#include <deque>
using namespace mscnav;
using namespace utiltool;

void output()
{
    std::ofstream ofs_imu("./data/imu.bin", std::fstream::binary);
    int count = 0;
    NavTime time(1984, 15000.0);
    ImuData imu(time);
    imu.gyro_ = {0.1, 0.1, 0.1};
    imu.acce_ = {0.3, 0.3, 0.3};
    while (count < 60 * 10)
    {
        ofs_imu.write(reinterpret_cast<char *>(&imu), sizeof(imu));
        time += 0.1;
        imu.set_time(time);
        count++;
    }
    ofs_imu.close();

    std::ifstream ifs_file("../data/imu_copy.bin", std::fstream::binary);
    while (!ifs_file.eof())
    {
        {
            ImuData imu_read;
            ifs_file.read(reinterpret_cast<char *>(&imu_read), sizeof(ImuData));
            // *imu_data = imu_read;
            ImuData::Ptr imu_data = std::make_shared<ImuData>(imu_read);
            LOG(ERROR) << "output imu " << (*imu_data);
        }
        LOG(INFO) << "test";
        LOG(INFO) << "test";
        LOG(INFO) << "test";
        LOG(INFO) << "test";
    }
}

int main(int argc, char const *argv[])
{
    Eigen::Vector3d v;
    v << 1, 1, 2;
    Eigen::Vector3d v2= v.array().pow(2);
    output();
    LogInit(argv[0], "./log/");
    ConfigInfo::Ptr ptr = ConfigInfo::GetInstance();
    ptr->open(argv[1]);
    FileGnssData::Ptr gnss_data = std::make_shared<FileGnssData>();
    FileImuData::Ptr imu_data = std::make_shared<FileImuData>();
    gnss_data->StartReadGnssData();
    imu_data->StartReadData();

    DataQueue::Ptr data_queue = std::make_shared<DataQueue>(gnss_data, imu_data);
    while (true)
    {
        std::deque<BaseData::bPtr> test_queue;
        GnssData::Ptr gnss = std::make_shared<GnssData>();
        ImuData::Ptr imu = std::make_shared<ImuData>();
        {
            BaseData::bPtr base = imu;
            test_queue.emplace_back(base);
        }
        imu = nullptr;
        BaseData::bPtr base = test_queue.front();
        test_queue.pop_front();
        base->set_time(NavTime::NowTime());
        imu = std::dynamic_pointer_cast<ImuData>(base);

        base = data_queue->GetData();
        if (base->get_type() == GNSSDATA)
        {
            gnss = std::dynamic_pointer_cast<GnssData>(base);
            LOG(INFO) << "GNSS " << gnss->get_time().Time2String("%4d %.3f", NavTime::GPSTIME) << std::endl;
            LOG(ERROR) << std::fixed << std::setw(5) << gnss << std::endl;
        }
        else if (base->get_type() == IMUDATA)
        {
            imu = std::dynamic_pointer_cast<ImuData>(base);
            LOG(INFO) << "IMU " << imu->get_time().Time2String("%4d %.3f", NavTime::GPSTIME) << std::endl;
            LOG(ERROR) << std::fixed << std::setw(5) << imu << std::endl;
        }
        else if (base->get_type() == DATAUNKOWN)
        {
            LOG(INFO) << "Finish " << std::endl;
            break;
        }
        else
        {
            LOG(ERROR) << "ERROR";
        }
    }

    return 0;
}
