/*
** gpsprocess.h for mscnav in /media/fwt/Data/program/mscnav/include/process
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 31 上午11:30:30 2019 little fang
** Last update Thu Jul 31 上午11:42:00 2019 little fang
*/

#ifndef GPS_PROCESS_H_
#define GPS_PROCESS_H_
#include "filter/navfilter.h"
#include "data/navdataque.h"
#include "navstruct.hpp"

namespace mscnav
{
class GpsProcess
{
public:
    GpsProcess(const KalmanFilter::Ptr &filter) : filter_{filter} {}
    ~GpsProcess() {}
    using Ptr = std::shared_ptr<GpsProcess>;

public:
    bool processing(const utiltool::GnssData::Ptr &gnss_data,
                           const utiltool::NavInfo &nav_info,
                           Eigen::VectorXd &dx);

private:
    // static utiltool::NavTime last_time_;
    // static bool first_processing;
    KalmanFilter::Ptr filter_;
};

} // namespace mscnav

#endif /* !GPS_PROCESS_H_ */
