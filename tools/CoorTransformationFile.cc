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
#include "gflags/gflags.h"
#include <Eigen/Dense>
#include <string>
#include <fstream>

using namespace utiltool::earth;

int main(int argc, char *argv[])
{
    std::string file_name = argv[1];
    std::ifstream ifs_coor_file(argv[1]);
    std::ofstream ofs_blh_file(file_name.append(".blh"));
    std::string line;
    while (!ifs_coor_file.eof())
    {
        std::getline(ifs_coor_file, line);
        auto dat = utiltool::TextSplit(line, "\\s+");
        if (dat.size() < 12)
            continue;
        Eigen::Vector3d pos(std::stod(dat[8]), std::stod(dat[9]), std::stod(dat[10]));
        Eigen::Vector3d blh = WGS84XYZ2BLH(pos);
        ofs_blh_file << dat[7] <<","<< std::fixed << std::setprecision(5)
                     << blh(1) * utiltool::constant::rad2deg << ","
                     << blh(0) * utiltool::constant::rad2deg << ","
                     << blh(2) << std::endl;
    }
    ifs_coor_file.close();
    ofs_blh_file.close();
    return 0;
}
