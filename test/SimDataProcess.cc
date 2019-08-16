/*
** SimDataProcess.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Fri Aug 16 下午3:49:43 2019 little fang
** Last update Sat Aug 16 下午9:54:18 2019 little fang
*/
#include "navtime.h"
#include "navbase.hpp"
#include "navstruct.hpp"
#include "constant.hpp"
#include "navearth.hpp"
#include <fstream>
#include <Eigen/Dense>
using namespace std;
using namespace utiltool;
using namespace utiltool::constant;

int main(int argc, char const *argv[])
{
    string path = argv[1];
    ifstream gps_data(path + "/gps-0.csv");
    ifstream gps_time(path + "/gps_time.csv");
    ifstream accel_data(path + "/accel-0.csv");
    ifstream gyro_data(path + "/gyro-0.csv");
    ifstream imu_time(path + "/time.csv");

    ofstream gps_file(path + "/gps.txt");
    ofstream imu_file(path + "/imu.bin", std::ofstream::binary);

    string line;
    getline(gps_data, line);
    getline(gps_time, line);
    getline(accel_data, line);
    getline(gyro_data, line);
    getline(imu_time, line);
    Eigen::Vector3d blh;
    while (!gps_data.eof())
    {
        line.clear();
        getline(gps_data, line);
        if (line.size() == 0)
            continue;
        auto data = TextSplit(line, ",");
        blh << stod(data[0]) * deg2rad, stod(data[1]) * deg2rad, stod(data[2]);
        line.clear();
        getline(gps_time, line);
        double second = stod(line);
        gps_file << "2000 " << std::setprecision(1) << std::setw(10) << second << "\t\t"
                 << std::fixed << setprecision(4) << earth::WGS84BLH2XYZ(blh).transpose()
                << " 1.0 1.0 1.0" << std::endl;
    }
    gps_data.close();
    gps_file.close();
    gps_time.close();

    Eigen::Vector3d gyro, accel;
    while (!gyro_data.eof())
    {
        line.clear();
        getline(gyro_data, line);
        if (line.size() == 0)
            continue;
        auto data = TextSplit(line, ",");
        gyro << stod(data[0]), stod(data[1]), stod(data[2]);

        line.clear();
        getline(accel_data, line);
        auto data2 = TextSplit(line, ",");
        accel << stod(data2[0]), stod(data2[1]), stod(data2[2]);

        line.clear();
        getline(imu_time, line);
        double second = stod(line);
        NavTime time(2000, second); //2000是随便假定的周
        ImuData imu(time);
        imu.acce_ = accel * 0.01;
        imu.gyro_ = gyro * deg2rad * 0.01;
        imu_file.write(reinterpret_cast<char *>(&imu), sizeof(imu));
        std::cout << imu.get_time() << std::endl;
    }
    return 0;
}
