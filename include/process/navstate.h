/*
** navstate.h for mscnav in /media/fwt/Data/program/mscnav/include/process
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 31 下午8:21:33 2019 little fang
** Last update Sat Aug 2 上午10:54:27 2019 little fang
*/

#ifndef PROCESS_STATE_H_
#define PROCESS_STATE_H_

#include "filter/navfilter.h"
#include "imu/navinitialized.h"
#include "imu/navmech.h"
#include "data/navdataque.h"
#include "gpsprocess.h"
#include "navconfig.hpp"

namespace mscnav
{
class State
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    ~State() {}
    using Ptr = std::shared_ptr<State>;
    static Ptr GetState();

private:
    State(){}
    State(State &&) = delete;
    State(const State &) = delete;
    State &operator=(State &&) = delete;
    State &operator=(const State &) = delete;
    static Ptr state_;

public:
    void StartProcessing();

private:
    bool InitializeState();
    void ReviseState(const Eigen::VectorXd &dx);

private:
    DataQueue::Ptr data_queue_;
    GpsProcess::Ptr gps_process_;
    KalmanFilter::Ptr filter_;
    ImuDataCollect::Ptr imu_data_;
    InitializedNav::Ptr initialize_nav_;
    GnssDataCollect::Ptr gnss_data_;
    utiltool::ConfigInfo::Ptr config_;
    utiltool::NavInfo nav_info_;
    utiltool::NavInfo nav_info_bak_;
    utiltool::NavTime latest_update_time_;
    Eigen::MatrixXd state_q_;

private:
    std::ofstream ofs_result_output_;
};
} // namespace mscnav

#endif /* !PROCESS_STATE_H_ */
