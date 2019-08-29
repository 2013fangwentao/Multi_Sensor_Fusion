/*
** NavFilter.cc for Test filter in /home/fwt/mypro/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue May 14 9:29:43 2019 little fang
** Last update Wed Jul 16 ??11:06:35 2019 little fang
*/
#include "filter/navfilter.h"
#include "navlog.hpp"
#include "navconfig.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace mscnav;
using namespace utiltool;

int main(int argc, const char *argv[])
{
  auto config = ConfigInfo::GetInstance();
  config->open("../config/configture.ini");
  LogInit(argv[0], "./log/");

  int index = 0;
  Eigen::VectorXd init_cov;
  init_cov <<0.8, 1, 3, 2.4;
  KalmanFilter filter(config->get<int>("filter_debug_log_enable"));
  filter.InitialStateCov(init_cov);
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(4, 4);
  Eigen::VectorXd vQ(4);
  vQ << 0.5, 4, 3, 1;
  Eigen::MatrixXd Q = vQ.asDiagonal();
  Eigen::VectorXd vR(4);
  vR << 0.3, 0.4, 2.3, 1.1;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd Z(4, 1);
  Z << 0.3, 0.3, 0.4, 0.5;
  NavTime time(2019, 1, 1, 0, 0, 0.0);
  while (index < 10)
  {
    filter.TimeUpdate(Phi, Q, time);
    time += 30;
    filter.MeasureUpdate(H, Z, R, time);
    index++;
  }
  auto &mat = filter.GetStateCov();
  LOG(INFO) << " Size: \n"
            << filter.GetStateSize() << std::endl;
  LOG(INFO) << " Cov: \n"
            << filter.GetStateCov() << std::endl;
  mat << 0.1, 3, 4, 2,
      0.5, 0.3, 0.9, 1,
      0.532, 1.90, 3.90, 3.2,
      1, 2, 3, 4;
  LOG(INFO) << " Cov1: \n"
            << filter.GetStateCov() << std::endl;
  auto mat2 = filter.GetStateCov();
  mat2(0, 0) = 20000;
  filter.EliminateIndex(0);
  LOG(INFO) << " Cov2: \n"
            << filter.GetStateCov() << std::endl;
  // filter.InsertIndex(1, init_cov);
  LOG(INFO) << " Cov3: \n"
            << filter.GetStateCov() << std::endl;
}
