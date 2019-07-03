/*
** filter.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** base class of kalman filter
** Started on  Tue Dec 17 下午3:02:10 2018 little fang
** Last update Thu Jul 3 13:13:04 2019 little fang
*/

#ifndef FILTER_H_
#define FILTER_H_
#include "navstruct.h"
#include "navtime.h"
#include <Eigen/Dense>
#include <fstream>
#include <vector>

namespace mscnav
{

class KalmanFilter
{
public:
  KalmanFilter(bool debug_log = false) : debug_log_(debug_log) {}
  virtual ~KalmanFilter() {}

public:
  bool InitialStateCov(std::vector<double> init_state_cov);
  bool TimeUpdate(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q, const utiltool::NavTime &time);
  Eigen::VectorXd MeasureUpdate(const Eigen::MatrixXd &H, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &R,
                                    const utiltool::NavTime &time);

private:
  Eigen::MatrixXd state_cov_;
  bool debug_log_ = false;
  std::ofstream debug_log_file_;
};

} // namespace mscnav
#endif /* !FILTER_H_ */
