/*
** NavPureMech.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Jul 16 下午10:20:02 2019 little fang
** Last update Thu Jul 17 下午5:44:58 2019 little fang
*/

#include "imu/navmech.h"
#include "navstruct.hpp"
#include "navlog.hpp"
#include "constant.hpp"
#include "navattitude.hpp"
#include "navearth.hpp"
#include "navtime.h"
#include <string>
#include <fstream>

using std::string;
using namespace mscnav;
using namespace utiltool;
using namespace utiltool::constant;
using namespace utiltool::attitude;
const double starttime = 91620.005;
const Eigen::Vector3d pos = earth::WGS84BLH2XYZ({23.1373950_deg, 113.37136499999_deg, 2.17499937627});
const Eigen::Vector3d vel = earth::CalCn2e(23.1373950_deg, 113.37136499999_deg) * Eigen::Vector3d{0.00017423244, -0.00032699472, 0.00024949312};
// const Eigen::Quaterniond quat = Euler2Quaternion(Euler{0.01083286617_deg, -2.14248721480_deg, -75.74984266986_deg});
const Euler euler{0.01083286617_deg, -2.14248721480_deg, -75.74984266986_deg};

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

ImuData &Convert(const IMU_DATA &imu, ImuData &imu_data)
{
    imu_data.acce_ = Eigen::Vector3d{imu.acce[0], imu.acce[1], imu.acce[2]};
    imu_data.gyro_ = Eigen::Vector3d{imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    NavTime time(2000, imu.time);
    // LOG(INFO) << imu.time << std::endl;
    imu_data.set_time(time);
    return imu_data;
}

int main(int argc, char const *argv[])
{
    LogInit(argv[0], "./log/");
    std::ofstream ofs_out("./log/result.txt");
    string imu_data_path = argv[1];
    FILE *imu_file = fopen(imu_data_path.c_str(), "rb");
    IMU_DATA curr_imu, pre_imu;
    ImuData curr_data, pre_data;
    while (!feof(imu_file))
    {
        int n = fread(&curr_imu, sizeof(IMU_DATA), 1, imu_file);
        if (curr_imu.time > starttime)
        {
            break;
        }
        pre_imu = curr_imu;
    }
    NavInfo curr_nav_info;

    Eigen::Matrix3d Cbn = Euler2RotationMatrix(euler);
    Eigen::Matrix3d Cne = earth::CalCn2e(23.1373950_deg, 113.37136499999_deg);
    Eigen::Matrix3d Cbe = earth::CalCn2e(23.1373950_deg, 113.37136499999_deg) * Cbn;

    curr_nav_info.quat_ = RotationMartix2Quaternion(Cbe);
    curr_nav_info.att_ = euler;
    curr_nav_info.pos_ = pos;
    curr_nav_info.vel_ = vel;
    Convert(pre_imu, pre_data);
    Convert(curr_imu, curr_data);
    Eigen::MatrixXd phi;
    while (!feof(imu_file))
    {
        curr_nav_info = navmech::MechanicalArrangement(pre_data, curr_data, curr_nav_info, phi);
        pre_data = curr_data;
        int n = fread(&curr_imu, sizeof(IMU_DATA), 1, imu_file);
        Convert(curr_imu, curr_data);
        ofs_out << std::fixed << std::setprecision(15) << curr_nav_info << std::endl;
        LOG(INFO) << curr_nav_info.time_ << std::endl;
    }

    return 0;
}
