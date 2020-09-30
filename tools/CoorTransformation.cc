/*
** NavCoorChange.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 14 下午4:49:44 2019 little fang
** Last update Thu Aug 14 下午5:44:59 2019 little fang
*/

#include "navbase.hpp"
#include "navearth.hpp"
#include "constant.hpp"
#include <gflags/gflags.h>
#include <Eigen/Dense>
#include <string>

using namespace utiltool::earth;

DEFINE_string(tran_type, "BLH2XYZ", "转换类型");
DEFINE_string(angle_type, "deg", "角度的单位：deg/rad");
DEFINE_string(XYZ, "0,0,0", "XYZ坐标");
DEFINE_string(BLH, "0,0,0", "BLH坐标");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "/*--------------------------- input ---------------------------*/ " << std::endl;
    std::cout << "tran_type: " << FLAGS_tran_type << std::endl;
    std::cout << "angle_type: " << FLAGS_angle_type << std::endl;
    std::cout << "XYZ: " << FLAGS_XYZ << std::endl;
    std::cout << "BLH: " << FLAGS_BLH << std::endl
              << std::endl;
    std::cout << "/*--------------------------- output ---------------------------*/ " << std::endl;
    if (FLAGS_tran_type == "BLH2XYZ")
    {
        auto BLH_str = utiltool::TextSplit(FLAGS_BLH, ",");
        if (BLH_str.size() != 3)
        {
            std::cout << "BLH error" << std::endl;
            exit(0);
        }
        Eigen::Vector3d blh;
        blh << std::stod(BLH_str[0]), std::stod(BLH_str[1]), std::stod(BLH_str[2]);
        if (FLAGS_angle_type == "deg")
        {
            blh.segment<2>(0) *= utiltool::constant::deg2rad;
        }
        std::cout << "XYZ: " << std::fixed << std::setprecision(4) << WGS84BLH2XYZ(blh).transpose() << std::endl;
    }
    else if (FLAGS_tran_type == "XYZ2BLH")
    {
        auto XYZ_str = utiltool::TextSplit(FLAGS_XYZ, ",");
        if (XYZ_str.size() != 3)
        {
            std::cout << "XYZ error" << std::endl;
            exit(0);
        }
        Eigen::Vector3d xyz{std::stod(XYZ_str[0]), std::stod(XYZ_str[1]), std::stod(XYZ_str[2])};
        auto blh = WGS84XYZ2BLH(xyz);
        std::cout << "BLH in rad: " << std::fixed << std::setprecision(13) << blh.segment<2>(0).transpose() << std::setprecision(4) << "\t" << blh(3) << std::endl;
        std::cout << "BLH in deg: " << std::fixed << std::setprecision(8) << blh.segment<2>(0).transpose() * utiltool::constant::rad2deg << "\t"
                  << std::setprecision(4) << blh(3) << std::endl;
    }
    else
    {
        std::cout << "tran_type error" << std::endl;
        exit(0);
    }
    google::ShutDownCommandLineFlags();
    return 0;
}
