/*
** NavFilter.cc for Test filter in /home/fwt/mypro/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue May 14 下午9:29:43 2019 little fang
** Last update Thu Jul 3 13:31:01 2019 little fang
*/
#include "filter/filter.h"
#include "navlog.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace mscnav;
using namespace mscnav::utiltool;

int main(int argc, const char *argv[])
{
  navloginit(argv[0], "./log/");
  int index = 0;
  std::vector<double> init_cov{0.8, 1, 3, 2.4};
  KalmanFilter filter;
  filter.InitialStateCov(init_cov);
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(3, 4);
  Eigen::VectorXd vQ(4);
  vQ << 0.5, 4, 3, 1;
  Eigen::MatrixXd Q = vQ.diagonal();
  Eigen::VectorXd vR(4);
  vR << 0.3, 0.4, 2.3, 1.1;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd Z(4, 1);
  Z << 0.3, 0.3, 0.4, 0.5;
  std::cout << Z << std::endl;
  NavTime time(2019, 1, 1, 0, 0, 0.0);
  while (index < 10)
  {
    filter.TimeUpdate(Phi, Q, time);
    time += 30;
    filter.MeasureUpdate(H, Z, R, time);
    index++;
  }
}
