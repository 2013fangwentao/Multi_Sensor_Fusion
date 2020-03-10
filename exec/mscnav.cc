/*
** mscnav.cc for mscnav in /media/fwt/Data/program/mscnav/exec
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Aug 25 下午10:49:42 2019 little fang
** Last update Thu Nov 6 下午8:22:36 2019 little fang
*/

#include "constant.hpp"
#include "process/navstate.h"
#include "navattitude.hpp"
#include "navconfig.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace mscnav;
using namespace utiltool;
using namespace utiltool::constant;
using namespace utiltool::attitude;

int main(int argc, char const *argv[])
{
    if (argc != 3)
    {
        std::cout << "Parameter error" << std::endl;
        std::cout << "[executable] [configure_file] [log_dir]" << std::endl;
        return 0;
    }
    utiltool::NavTime time_start = utiltool::NavTime::NowTime();
    ConfigInfo::Ptr config = ConfigInfo::GetInstance();
    config->open(argv[1]);
    LogInit(argv[0], argv[2], 2);

    State::Ptr state = State::GetState();
    state->StartProcessing();
    utiltool::NavTime time_end = utiltool::NavTime::NowTime();
    LOG(INFO) << "consumed time is  " << (time_end - time_start) << " s" << std::endl;
    return 0;
}
