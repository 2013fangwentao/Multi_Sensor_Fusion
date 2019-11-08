/*
** preprocess_kitti_data.cc for mscnav in /media/fwt/Data/program/mscnav/tools
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Aug 25 下午5:35:25 2019 little fang
** Last update Tue Aug 26 上午9:30:22 2019 little fang
*/

#include "navbase.hpp"
#include "navstruct.hpp"
#include "navtime.h"
#include "navearth.hpp"
#include "constant.hpp"
#include <fstream>
#include <string>
#include <Eigen/Dense>

int main(int argc, char const *argv[])
{
    using namespace utiltool;
    using namespace std;
    using namespace utiltool::earth;
    using namespace utiltool::constant;
    using Eigen::Vector3d;

    if (argc != 3)
    {
        std::cout << "uasge: exexutable data.txt output_path" << std::endl;
        return 0;
    }
    ifstream ifs_merge_data(argv[1]);
    string output_path = argv[2];
    if (!ifs_merge_data.good())
    {
        std::cout << "read file failed \t " << argv[1] << std::endl;
        return 0;
    }
    ofstream ofs_gnss_file(output_path + "/gnss.txt");
    ofstream ofs_ref_file(output_path + "/reference.txt");
    ofstream ofs_imu_file(output_path + "/imu.bin", std::fstream::binary);
    ofstream ofs_imutxt_file(output_path + "/imu.txt");

    double dt = 0.0;
    NavTime bak_time = NavTime::NowTime();
    Vector3d bak_gyro, bak_accel;
    while (!ifs_merge_data.eof())
    {
        std::string line_data;
        std::getline(ifs_merge_data, line_data);
        auto data = TextSplit(line_data, "\\s+");
        if (data.size() < 15)
            continue;
        NavTime time(stoi(data[0]),
                     stoi(data[1]),
                     stoi(data[2]),
                     stoi(data[3]),
                     stoi(data[4]),
                     stod(data[5]));
        Vector3d BLH{stod(data[6]) * deg2rad,
                     stod(data[7]) * deg2rad,
                     stod(data[8])};
        Vector3d XYZ = WGS84BLH2XYZ(BLH);

        Vector3d gyro = {stod(data[6 + 3]),
                         stod(data[7 + 3]),
                         stod(data[8 + 3])};
        Vector3d accel = {stod(data[6 + 6]),
                          stod(data[7 + 6]),
                          stod(data[8 + 6])};
        Vector3d att = {stod(data[6 + 9]),
                        stod(data[7 + 9]),
                        stod(data[8 + 9])};
        dt = time - bak_time;
        bak_time = time;

        if (dt < 0)
        {
            bak_gyro = gyro;
            bak_accel = accel;
            continue;
        }

        ImuData imu(time);
        imu.acce_ = -1 * (accel + bak_accel) * dt / 2.0;
        imu.acce_[0] *= -1;

        imu.gyro_ = -1 * (gyro + bak_gyro) * dt / 2.0;
        imu.gyro_[0] *= -1;

        std::cout << time << std::endl;

        ofs_imutxt_file << time.GpsWeek() << "\t" << std::fixed
                        << std::setprecision(5) << time.SecondOfWeek() << "\t"
                        << imu.gyro_.transpose() << "\t" << imu.acce_.transpose() << std::endl;

        ofs_imu_file.write(reinterpret_cast<char *>(&imu), sizeof(imu));

        ofs_ref_file << time.GpsWeek() << "\t" << std::fixed
                     << std::setprecision(5) << time.SecondOfWeek()
                     << "\t" << XYZ.transpose() << "\t"
                     << (att * rad2deg).transpose() << std::endl;

        if (int(time.Second() * 10) % 10 == 0)
        {
            ofs_gnss_file << time.GpsWeek() << "\t" << std::fixed
                          << std::setprecision(5) << time.SecondOfWeek() << "\t"
                          << XYZ.transpose() << "\t0.5\t0.5\t0.5" << std::endl;
        }
    }
    ifs_merge_data.close();
    ofs_gnss_file.close();
    ofs_imu_file.close();
    ofs_ref_file.close();
    std::cout << "finish" << std::endl;
    return 0;
}
