/*
** RefProcess.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Fri Aug 16 下午8:43:42 2019 little fang
** Last update Sat Aug 16 下午9:24:36 2019 little fang
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

void read_ref_pos(Eigen::Vector3d &ref_pos, double *ref_time, ifstream &ifs_ref_pos, ifstream &ifs_time_ref_pos)
{
    string line;
    getline(ifs_time_ref_pos, line);
    *ref_time = stod(line);
    getline(ifs_ref_pos, line);
    auto data = TextSplit(line, ",");
    ref_pos << stod(data[0]) * deg2rad, stod(data[1]) * deg2rad, stod(data[2]);
}

void read_result_pos(Eigen::Vector3d &result_pos, double *result_time, ifstream &ifs_loosecouple_result)
{
    string line;
    getline(ifs_loosecouple_result, line);
    auto data = TextSplit(line, "\\s+");
    *result_time = stod(data[3]);
    result_pos << stod(data[4]), stod(data[5]), stod(data[6]);
}

int main(int argc, char const *argv[])
{
    if (argc != 5)
    {
        std::cout << "usage: executable [sim_ref_pos] [time_of_ref_pos] [loose_couple_gnssins_reult] [residual_file]" << std::endl;
        navexit();
    }
    ifstream ifs_ref_pos(argv[1]);
    ifstream ifs_time_ref_pos(argv[2]);
    ifstream ifs_loosecouple_result(argv[3]);
    ofstream ofs_residual_result(argv[4]);

    string line;
    getline(ifs_ref_pos, line);
    getline(ifs_time_ref_pos, line);

    Eigen::Vector3d ref_pos{0, 0, 0};
    Eigen::Vector3d result_pos{0, 0, 0};
    double ref_time = 0, result_time = 0;

    read_ref_pos(ref_pos, &ref_time, ifs_ref_pos, ifs_time_ref_pos);
    read_result_pos(result_pos, &result_time, ifs_loosecouple_result);

    while (!ifs_time_ref_pos.eof() && !ifs_loosecouple_result.eof())
    {
        if (result_time == ref_time)
        {
            auto xyz = earth::WGS84BLH2XYZ(ref_pos);
            ofs_residual_result << std::setprecision(3) << ref_time << "  " << std::fixed << std::setprecision(4) << (xyz - result_pos).transpose() << std::endl;
            cout << ref_time << std::endl;
            read_ref_pos(ref_pos, &ref_time, ifs_ref_pos, ifs_time_ref_pos);
            read_result_pos(result_pos, &result_time, ifs_loosecouple_result);
        }
        else if (result_time > ref_time)
        {
            read_ref_pos(ref_pos, &ref_time, ifs_ref_pos, ifs_time_ref_pos);
        }
        else
        {
            read_result_pos(result_pos, &result_time, ifs_loosecouple_result);
        }
    }
    ifs_ref_pos.close();
    ifs_time_ref_pos.close();
    ifs_loosecouple_result.close();
    ofs_residual_result.close();
    return 0;
}
