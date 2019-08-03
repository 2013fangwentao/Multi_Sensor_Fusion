/*
** NavProcessing.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Sat Aug 3 下午1:45:12 2019 little fang
** Last update Sat Aug 3 下午1:45:12 2019 little fang
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
    if(argc!=3)
    {
        std::cout << "Parameter error" << std::endl;
        std::cout << "[executable] [configure_file] [log_dir]" << std::endl;
        return 0;
    }
    ConfigInfo::Ptr config = ConfigInfo::GetInstance();
    config->open(argv[1]);
    LogInit(argv[0],argv[2],0);

    State::Ptr state = State::GetState();
    state->StartProcessing();
    return 0;
}
