
#include "navconfig.hpp"
#include "imu/navinitialized.h"
#include <deque>
using namespace mscnav;
using namespace utiltool;

struct IMU_DATA
{
    double time;
    double gyro[3];
    double acce[3];
    IMU_DATA()
    {
        time = 0.0;
        memset(gyro, 0x0, sizeof(double) * 3);
        memset(acce, 0x0, sizeof(double) * 3);
    }
};

int main(int argc, char **argv)
{
    FILE *fp_imu;
    IMU_DATA imu;
    fp_imu = fopen("/media/fwt/Data/program/mscnav/data/alignmentdata/00010190.imu", "rb");
    std::ofstream fps_imu("/media/fwt/Data/program/mscnav/data/alignmentdata/imu_file.bin", std::fstream::binary);
    while (!feof(fp_imu))
    {
        int n = fread(&imu, sizeof(IMU_DATA), 1, fp_imu);
        NavTime time(1984, imu.time);
        ImuData imu_data(time);
        imu_data.gyro_ = Eigen::Vector3d{imu.gyro[0], imu.gyro[1], imu.gyro[2]};
        imu_data.acce_ = Eigen::Vector3d{imu.acce[0], imu.acce[1], imu.acce[2]};
        fps_imu.write(reinterpret_cast<char *>(&imu_data), sizeof(imu_data));
    }
    fclose(fp_imu);
    fps_imu.close();
    /*   std::ifstream ifs_file("../data/imu_file.bin", std::fstream::binary);
    while (!ifs_file.eof())
    {
        {
            ImuData imu_read;
            ifs_file.read(reinterpret_cast<char *>(&imu_read), sizeof(ImuData));
            ImuData::Ptr imu_data = std::make_shared<ImuData>(imu_read);
            LOG(ERROR) << "output imu " << (*imu_data);
        }
        LOG(INFO) << "test";
        LOG(INFO) << "test";
        LOG(INFO) << "test";
        LOG(INFO) << "test";
    } */
    // LogInit(argv[0], "./log/");
    // ConfigInfo::Ptr ptr = ConfigInfo::GetInstance();
    // ptr->open(argv[1]);
    // FileGnssData::Ptr gnss_data = std::make_shared<FileGnssData>();
    // FileImuData::Ptr imu_data = std::make_shared<FileImuData>();
    // gnss_data->StartReadGnssData();
    // imu_data->StartReadData();

    // DataQueue::Ptr data_align = std::make_shared<DataQueue>(gnss_data, imu_data);
    // InitializedNav align(data_align);
    // NavInfo nav_info;
    // align.StartAligning(nav_info);
    // if(nav_info.att_[2]<0)
    // {
    //     nav_info.att_[2]=nav_info.att_[2]+2*3.1415926;
    // }
    // LOG(INFO)<<nav_info.time_<<std::setprecision(10)<<nav_info.att_.transpose() * constant::rad2deg<<std::endl;
    // // printf(" %f %f %f ",  nav_info.att_[0], nav_info.att_[1], nav_info.att_[2]);
    // navexit(0);
    return 0;
}
