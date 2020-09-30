/*
** RawImuToImuData.cc for mscnav in /media/fwt/Data/program/mscnav/tools
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Aug 26 下午2:22:04 2019 little fang
** Last update Thu Jan 15 下午2:53:59 2020 little fang
*/

#include "navtime.h"
#include "navbase.hpp"
#include "gflags/gflags.h"
#include <iostream>
#include <fstream>
using namespace utiltool;

DEFINE_string(ImuRaw, "./imu.imu", "原始类型imu数据");
DEFINE_string(ImuData, "./imu.bin", "转换结果文件");

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

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    FILE *fp_imu;
    IMU_DATA imu;
    fp_imu = fopen(FLAGS_ImuRaw.c_str(), "rb");
    if(!fp_imu)
    {
        std::cout << FLAGS_ImuRaw << " Error" << std::endl;
        return 0;
    }
    std::ofstream fps_imu(FLAGS_ImuData, std::fstream::binary);
    while (!feof(fp_imu))
    {
        int n = fread(&imu, sizeof(IMU_DATA), 1, fp_imu);
        NavTime time(2086, imu.time);
        ImuData imu_data(time);
        imu_data.gyro_ = Eigen::Vector3d{imu.gyro[0], imu.gyro[1], imu.gyro[2]};
        imu_data.acce_ = Eigen::Vector3d{imu.acce[0], imu.acce[1], imu.acce[2]};
        fps_imu.write(reinterpret_cast<char *>(&imu_data), sizeof(imu_data));
    }
    std::cout << "finish" << std::endl;
    fclose(fp_imu);
    fps_imu.close();
}
