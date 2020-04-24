/*
** filter.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/src/filter
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Dec 17 下午3:03:16 2018 little fang
** Last update Sat Apr 24 下午3:38:38 2020 little fang
*/

#include "filter/navfilter.h"
#include "navconfig.hpp"
#include "navlog.hpp"
#include "navstruct.hpp"
#include "navattitude.hpp"
#include <iomanip>

namespace mscnav
{
using namespace utiltool;

bool KalmanFilter::InitialStateCov(const Eigen::VectorXd &init_state_cov)
{
  int size_vector = init_state_cov.size();
  if (size_vector == 0)
  {
    LOG(FATAL) << ("初始化方差错误,状态数量为 0 !") << std::endl;
    return false;
  }
  Eigen::VectorXd P0 = init_state_cov.array().pow(2);
  state_cov_ = P0.asDiagonal();
  LOG(INFO) << "初始化方差完成, 状态维度: " << size_vector << std::endl;
  if (debug_log_)
  {
    ConfigInfo::Ptr getconfig = ConfigInfo::GetInstance();

    std::string debug_info_path = getconfig->get<std::string>("result_output_path") + "/filter";
    debug_info_path.append(".log-" + (utiltool::NavTime::NowTime()).Time2String("%04d-%02d-%02d-%02d-%02d-%4.1f"));
    debug_log_file_.open(debug_info_path);
    LOG(INFO) << "filter_debug_cov_file: " << debug_info_path << std::endl;

    if (!debug_log_file_.good())
    {
      LOG(INFO) << ("Open filter debug file failed! ") << std::endl;
    }
    debug_log_file_ << "intial state covarance: " << std::endl
                    << state_cov_ << std::endl;
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
  if (Phi.rows() != state_index_.total_state || Q.rows() != state_index_.total_state)
  {
    LOG(ERROR) << ("the Dimension of Phi/Q is difference from IMU State_cov") << std::endl;
    return false;
  }

  if (state_index_.camera_state_index.size() == 0)
  {
    state_cov_ = Phi * state_cov_ * Phi.transpose() + Q;
  }
  else
  {
    //* msckf 时间更新
    //* 更新IMU部分的状态方差信息
    auto &Nstate = state_index_.total_state;
    state_cov_.block(0, 0, Nstate, Nstate) =
        Phi * state_cov_.block(0, 0, Nstate, Nstate) * Phi.transpose() + Q;
    // state_cov_ = Phi * state_cov_ * Phi.transpose() + Q;

    //* 更新IMU和Camera的协方差
    int Ncamstate = state_index_.camera_state_index.size() * 6;
    state_cov_.block(0, Nstate, Nstate, Ncamstate) =
        Phi * state_cov_.block(0, Nstate, Nstate, Ncamstate);
    state_cov_.block(Nstate, 0, Ncamstate, Nstate) =
        state_cov_.block(Nstate, 0, Ncamstate, Nstate) * Phi.transpose();
  }
  if (debug_log_)
  {
    debug_log_file_ << std::endl
                    << std::fixed << time.Time2String() << std::endl;
    debug_log_file_ << std::setprecision(18) << "state cov" << std::endl
                    << state_cov_ << std::endl
                    << "PHI " << std::endl
                    << Phi << std::endl
                    << " Q " << std::endl
                    << Q << std::endl;
  }
}

Eigen::VectorXd KalmanFilter::MeasureUpdate(const Eigen::MatrixXd &H, const Eigen::VectorXd &Z, const Eigen::MatrixXd &R,
                                            const utiltool::NavTime &time)
{
  double second = time.SecondOfDay();
  int int_second = int(second * 100);
  if (debug_log_ /*&& int_second % 100 == 0*/)
  {
    debug_log_file_ << "\n"
                    << std::fixed << time.Time2String() <<" "<<time.SecondOfWeek()<< std::endl;
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
  Eigen::VectorXd deltaX = K * Z;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_cov_.rows(), state_cov_.cols());
  // state_cov_ = (I - K * H) * state_cov_ * (I - K * H).transpose() + K * R * K.transpose();
  state_cov_ = (I - K * H) * state_cov_;
  Eigen::MatrixXd state_cov_fixed = (state_cov_ + state_cov_.transpose()) / 2.0;
  state_cov_ = state_cov_fixed; //* 保持方差正定
  if (debug_log_ /*&& int_second % 100 == 0*/)
  {
    debug_log_file_ << std::setprecision(8) << "after update state cov" << std::endl
                    << state_cov_ << std::endl
                    << "deltaX " << std::endl
                    << deltaX << std::endl;
  }
  return deltaX;
}

/**
 * @brief  return the state cov which can be modify
 * @note   
 * @retval 
 */
Eigen::MatrixXd &KalmanFilter::GetStateCov()
{
  return state_cov_;
}

/**
 * @brief  return the state cov const
 * @note   
 * @retval 
 */
Eigen::MatrixXd KalmanFilter::GetStateCov() const
{
  return state_cov_;
}

/**
 * @brief  
 * @note   
 * @retval 
 */
StateIndex &KalmanFilter::GetStateIndex()
{
  return state_index_;
}
/**
 * @brief  return the state index
 * @note   
 * @retval 
 */
StateIndex KalmanFilter::GetStateIndex() const
{
  return state_index_;
}
/**
 * @brief  get the count of state cov
 * @note   
 * @retval 
 */
size_t KalmanFilter::GetStateSize() const
{
  return state_cov_.rows();
}

/**
 * @brief  eliminate the elemt of state cov
 * @note   
 * @param  index: start index (from 0)
 * @param  count: total eliminate rows and cols (default=1)
 * @retval  true: eliminate successed
 *          false: index beyond the total size
 */
bool KalmanFilter::EliminateIndex(size_t index, size_t count)
{
  if (!(index < state_cov_.rows()))
  {
    LOG(WARNING) << "Eliminate index failed because of the index beyond the total size" << std::endl;
    return false;
  }
  if ((index + count) >= state_cov_.rows())
  {
    state_cov_.conservativeResize(index, index);
    return true;
  }
  size_t row = state_cov_.rows();
  size_t block_size = row - count - index;
  state_cov_.block(0, index, index, block_size) = state_cov_.block(0, index + count, index, block_size);
  state_cov_.block(index, 0, block_size, index) = state_cov_.block(index + count, 0, block_size, index);
  state_cov_.block(index, index, block_size, block_size) = state_cov_.block(index + count, index + count, block_size, block_size);
  state_cov_.conservativeResize(row - count, row - count);
  return true;
}

/**
 * @brief  insert state into the state cov 
 * @note   //! attention the inserted data will insert before of the start_index
 * @param  start_index: 
 * @param  init_cov: initial cov 
 * @retval 
 */
bool KalmanFilter::InsertIndex(size_t start_index, std::vector<double> init_cov)
{
  if (init_cov.size() == 0)
  {
    LOG(WARNING) << "insert state failed because of the count of init cov is zero" << std::endl;
    return false;
  }
  size_t row = state_cov_.rows();
  size_t count = init_cov.size();
  state_cov_.conservativeResize(count + row, count + row);
  state_cov_.rightCols(count) = state_cov_.middleCols(start_index, count);
  state_cov_.bottomRows(count) = state_cov_.middleRows(start_index, count);
  state_cov_.middleCols(start_index, count) *= 0;
  state_cov_.middleRows(start_index, count) *= 0;
  int index = 0;
  for (auto num : init_cov)
  {
    state_cov_(start_index + index, start_index + index) = num * num;
    index++;
  }
  return true;
}

void KalmanFilter::ReviseState(utiltool::NavInfo &nav_info, const Eigen::VectorXd &dx)
{
  using namespace utiltool;
  static ConfigInfo::Ptr config = ConfigInfo::GetInstance();

  // static std::string output_path = (config->get<std::string>("result_output_path")) + ("/dx.log");
  // static std::ofstream ofs_dx_log(output_path);
  // ofs_dx_log << std::fixed << std::setprecision(5) << nav_info.time_.SecondOfWeek()
  //            << " " << dx.transpose() << std::endl;

  nav_info.pos_ -= dx.segment<3>(state_index_.pos_index_);
  nav_info.vel_ -= dx.segment<3>(state_index_.vel_index_);
  Eigen::Quaterniond q_update = attitude::RotationVector2Quaternion(dx.segment<3>(state_index_.att_index_));
  nav_info.quat_ = q_update * nav_info.quat_;
  nav_info.quat_.normalize();
  NormalizeAttitude(nav_info);
  nav_info.gyro_bias_ += dx.segment<3>(state_index_.gyro_bias_index_);
  nav_info.acce_bias_ += dx.segment<3>(state_index_.acce_bias_index_);
  if (config->get<int>("evaluate_imu_scale") != 0)
  {
    nav_info.gyro_scale_ += dx.segment<3>(state_index_.gyro_scale_index_);
    nav_info.acce_scale_ += dx.segment<3>(state_index_.acce_scale_index_);
  }
}

} // namespace mscnav
