/*
** filter.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** base class of kalman filter
** Started on  Tue Dec 17 下午3:02:10 2018 little fang
** Last update Sat Aug 9 下午2:44:29 2019 little fang
*/

#ifndef FILTER_H_
#define FILTER_H_
#include "navstruct.hpp"
#include "navtime.h"
#include <Eigen/Dense>
#include <fstream>
#include <vector>

namespace mscnav
{


class KalmanFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  KalmanFilter(bool debug_log = false) : debug_log_(debug_log) {}
  virtual ~KalmanFilter() {}
  using Ptr = std::shared_ptr<KalmanFilter>;

public:
  bool InitialStateCov(const Eigen::VectorXd &init_state_cov);
  virtual bool TimeUpdate(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q, const utiltool::NavTime &time);
  virtual Eigen::VectorXd MeasureUpdate(const Eigen::MatrixXd &H, const Eigen::VectorXd &Z, const Eigen::MatrixXd &R,
                                        const utiltool::NavTime &time);
  virtual void ReviseState(utiltool::NavInfo& nav_info, const Eigen::VectorXd &dx);

public:
  Eigen::MatrixXd GetStateCov() const;
  Eigen::MatrixXd &GetStateCov();
  utiltool::StateIndex GetStateIndex() const;
  utiltool::StateIndex &GetStateIndex();
  size_t GetStateSize() const;
  bool EliminateIndex(size_t index, size_t count = 1);
  bool InsertIndex(size_t start_index, std::vector<double> init_cov);

protected:
  Eigen::MatrixXd state_cov_;
  bool debug_log_ = false;
  std::ofstream debug_log_file_;
  utiltool::StateIndex state_index_;
};

} // namespace mscnav
#endif /* !FILTER_H_ */
