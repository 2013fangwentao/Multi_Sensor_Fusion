#include "process/gpsprocess.h"
#include "navconfig.hpp"
#include "navearth.hpp"
using namespace utiltool::earth;
using namespace utiltool;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace mscnav
{
bool GpsProcess::processing(const utiltool::GnssData::Ptr &gnss_data,
                            const utiltool::NavInfo &nav_info,
                            Eigen::VectorXd &dx)
{
    ConfigInfo::Ptr config = ConfigInfo::GetInstance();
    static double data_rate = config->get<double>("data_rate");
    static bool is_use_vel = config->get<int>("data_rate");
    static const StateIndex index = filter_->GetStateIndex();
    double dt = gnss_data->get_time() - nav_info.time_;
    if (abs(dt) > 1.0 / data_rate)
    {
        LOG(ERROR) << "The time of gnss data and navinfo is not suitable !!!" << std::endl;
    }
    Vector3d pos = gnss_data->pos_ - nav_info.vel_ * dt; //* 补偿时间差带来的位置变化
    size_t state_size = filter_->GetStateCov().cols();
    MatrixXd Hmat = MatrixXd::Zero(3, state_size);
    VectorXd Zmat = VectorXd::Zero(3);
    Vector3d pos_std_pow = (gnss_data->pos_std_).array().pow(2);
    MatrixXd Rmat = pos_std_pow.asDiagonal();
    Zmat.segment<3>(0) = CorrectLeverarmPos(nav_info) - pos; 
    Hmat.block<3, 3>(0, index.pos_index_) = Matrix3d::Identity();
    Hmat.block<3, 3>(0, index.att_index_) = skew(nav_info.rotation_ * nav_info.leverarm_);
    dx = filter_->MeasureUpdate(Hmat, Zmat, Rmat, gnss_data->get_time());
    return true;
}
} // namespace mscnav