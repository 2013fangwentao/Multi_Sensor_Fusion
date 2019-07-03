/*
** filter.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/src/filter
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Dec 17 下午3:03:16 2018 little fang
** Last update Thu Jul 3 13:34:54 2019 little fang
*/

#include "filter/filter.h"
#include "navconfig.hpp"
#include "navlog.hpp"
#include "navstruct.h"
#include <iomanip>

namespace mscnav
{
using namespace utiltool;

bool KalmanFilter::InitialStateCov(std::vector<double> init_state_cov)
{
  int size_vector = init_state_cov.size();
  if (size_vector == 0)
  {
    naverrorlog("初始化方差错误,状态数量为 0 !");
    return false;
  }
  state_cov_ = Eigen::MatrixXd::Identity(size_vector, size_vector);
  for (int i = 0; i < size_vector; ++i)
  {
    state_cov_(i, i) = pow(init_state_cov[i], 2);
  }
  navinfolog("初始化方差完成, 状态维度: %02d", size_vector);
  if (debug_log_)
  {
    ConfigInfo::Ptr getconfig = ConfigInfo::GetInstance();
    debug_log_file_.open(getconfig->get<std::string>("filter_debug_cov_file"));
    if (!debug_log_file_)
    {
      naverrorlog("Open filter debug file failed!");
    }
  }
  return true;
}

/**
 * @brief  Kalman滤波量测更新,更新状态方差
 * @note
 * @param  &Phi:
 * @param  &Q:
 * @retval
 */
bool KalmanFilter::TimeUpdate(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q, const utiltool::NavTime &time)
{
  if (Phi.cols() != state_cov_.rows() || Phi.rows() != state_cov_.rows())
  {
    navwarnlog("the Dimension of Phi is difference from State_cov");
    return false;
  }
  if (Q.rows() != state_cov_.rows())
  {
    navwarnlog("the Dimension of Q is difference from State_cov");
    return false;
  }
  state_cov_ = Phi * state_cov_ * Phi.transpose() + Q;
  if (debug_log_)
  {
    debug_log_file_ << std::fixed << time.Time2String() << std::endl;
    debug_log_file_ << std::setprecision(8) << "state cov\n" << state_cov_ << std::endl << "PHI \n" << Phi << "\n Q \n" << Q;
  }
}

Eigen::VectorXd KalmanFilter::MeasureUpdate(const Eigen::MatrixXd &H, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &R,
                                            const utiltool::NavTime &time)
{
  if (debug_log_)
  {
    debug_log_file_ << std::fixed << time.Time2String() << std::endl;
    debug_log_file_ << std::setprecision(8) << "state cov" << std::endl
                    << state_cov_ << std::endl
                    << "H " << std::endl
                    << H << std::endl
                    << " Z " << std::endl
                    << Z.transpose() << std::endl
                    << " R " << std::endl
                    << R << std::endl;
  }
  Eigen::MatrixXd K = state_cov_ * H.transpose() * ((H * state_cov_ * H.transpose() + R).inverse());
  Eigen::MatrixXd deltaX = K * Z;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_cov_.rows(), state_cov_.cols());
  state_cov_ = (I - K * H) * state_cov_ * (I - K * H).transpose() + K * R * K.transpose();
  if (debug_log_)
  {
    debug_log_file_ << std::setprecision(8) << "after update state cov" << std::endl
                    << state_cov_ << std::endl
                    << "deltaX " << std::endl
                    << deltaX << std::endl;
  }
  return deltaX.col(0);
}

} // namespace mscnav
