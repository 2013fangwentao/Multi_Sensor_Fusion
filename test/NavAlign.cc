
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
    fp_imu = fopen("/home/dcq/program/data/alignmentdata/00010190.imu", "rb");
    std::ofstream fps_imu("../data/imu_file.bin", std::fstream::binary);
    while (!feof(fp_imu))
    {
        int n = fread(&imu, sizeof(IMU_DATA), 1, fp_imu);
        NavTime time(1984, imu.time);
        ImuData imu_data(time);
        imu_data.gyro_ = Eigen::Vector3d{imu.gyro[0], imu.gyro[1], imu.gyro[2]};
        imu_data.acce_ = Eigen::Vector3d{imu.acce[0], imu.acce[1], imu.acce[2]};
        ;
        fps_imu.write(reinterpret_cast<char *>(&imu_data), sizeof(imu_data));
    }
    fclose(fp_imu);
    fps_imu.close();
    std::ifstream ifs_file("../data/imu_file.bin", std::fstream::binary);
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
    }
    InitializedNav align();
    NavInfo nav_info;
    align().StartAligning(nav_info);
    return 0;
}