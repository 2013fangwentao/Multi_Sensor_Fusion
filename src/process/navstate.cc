#include "process/navstate.h"
#include "navconfig.hpp"
#include "navattitude.hpp"
#include "navearth.hpp"

using namespace utiltool;
using namespace constant;

namespace mscnav
{

State::Ptr State::state_(new State());

State::Ptr State::GetState()
{
    return state_;
}
/**
 * @brief  初始化
 * @note   
 * @retval 
 */
bool State::InitializeState()
{
    config_ = ConfigInfo::GetInstance();

    /* 数据相关内容 */
    bool gnss_log_enable = (config_->get<int>("gnsslog_enable") != 0);
    camera_enable = (config_->get<int>("camera_enable") != 0);
    gnss_data_ = std::make_shared<FileGnssData>(gnss_log_enable);
    if (!gnss_data_->StartReadGnssData())
    {
        navexit();
    }

    imu_data_ = std::make_shared<FileImuData>();
    if (!imu_data_->StartReadData())
    {
        navexit();
    }

    if (camera_enable)
    {
        camera_data_ = std::make_shared<FileCameraData>();
        if (!camera_data_->StartReadCameraData())
        {
            navexit();
        }
    }

    data_queue_ = std::make_shared<DataQueue>(gnss_data_, imu_data_, camera_data_);

    /*计算相关内容 */
    bool filter_debug_log = (config_->get<int>("filter_debug_log_enable") != 0);
    filter_ = std::make_shared<KalmanFilter>(filter_debug_log);
    initialize_nav_ = std::make_shared<InitializedNav>(data_queue_);
    gps_process_ = std::make_shared<GpsProcess>(filter_);

    if (camera_enable)
    {
        msckf_process_ = std::make_shared<camera::MsckfProcess>(filter_);
    }

    Eigen::VectorXd initial_Pvariance;
    if (initialize_nav_->StartAligning(nav_info_)) //获取初始的状态
    {
        LOG(INFO) << "Initialize the navigation information successed" << std::endl;
    }
    else
    {
        LOG(ERROR) << "Initialize the navigation information failed" << std::endl;
        return false;
    }

    initialize_nav_->SetStateIndex(filter_->GetStateIndex());                                                        //设置状态量对应的索引
    initial_Pvariance = initialize_nav_->SetInitialVariance(initial_Pvariance, nav_info_, filter_->GetStateIndex()); //设置初始方差信息
    filter_->InitialStateCov(initial_Pvariance);
    LOG(INFO) << "Initialized state successed" << std::endl;
    latest_update_time_ = nav_info_.time_;

    /**赋值小q 一定要注意单位 */
    const StateIndex &index = filter_->GetStateIndex();
    Eigen::VectorXd state_q_tmp = Eigen::VectorXd::Zero(filter_->GetStateSize());

    std::vector<double> tmp_value = config_->get_array<double>("position_random_walk");
    state_q_tmp.segment<3>(index.pos_index_)
        << tmp_value.at(0),
        tmp_value.at(1),
        tmp_value.at(2);

    tmp_value = config_->get_array<double>("velocity_random_walk");
    state_q_tmp.segment<3>(index.vel_index_)
        << tmp_value.at(0) / 60.0,
        tmp_value.at(1) / 60.0,
        tmp_value.at(2) / 60.0;

    tmp_value = config_->get_array<double>("attitude_random_walk");
    state_q_tmp.segment<3>(index.att_index_)
        << tmp_value.at(0) * deg2rad / 60.0,
        tmp_value.at(1) * deg2rad / 60.0,
        tmp_value.at(2) * deg2rad / 60.0;

    tmp_value = config_->get_array<double>("gyro_bias_std");
    state_q_tmp.segment<3>(index.gyro_bias_index_)
        << tmp_value.at(0) * dh2rs,
        tmp_value.at(1) * dh2rs,
        tmp_value.at(2) * dh2rs;

    tmp_value = config_->get_array<double>("acce_bias_std");
    state_q_tmp.segment<3>(index.acce_bias_index_)
        << tmp_value.at(0) * constant_mGal,
        tmp_value.at(1) * constant_mGal,
        tmp_value.at(2) * constant_mGal;

    if (config_->get<int>("evaluate_imu_scale") != 0)
    {
        tmp_value = config_->get_array<double>("gyro_scale_std");
        state_q_tmp.segment<3>(index.gyro_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;

        tmp_value = config_->get_array<double>("acce_scale_std");
        state_q_tmp.segment<3>(index.acce_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;
    }
    state_q_tmp = state_q_tmp.array().pow(2);
    double gbtime = config_->get<double>("corr_time_of_gyro_bias") * constant_hour;
    double abtime = config_->get<double>("corr_time_of_acce_bias") * constant_hour;

    state_q_tmp.segment<3>(index.gyro_bias_index_) *= (2.0 / gbtime);
    state_q_tmp.segment<3>(index.acce_bias_index_) *= (2.0 / abtime);

    if (config_->get<double>("evaluate_imu_scale") != 0)
    {
        double gstime = config_->get<double>("corr_time_of_gyro_scale") * constant_hour;
        double astime = config_->get<double>("corr_time_of_acce_scale") * constant_hour;
        state_q_tmp.segment<3>(index.gyro_scale_index_) *= (2.0 / gstime);
        state_q_tmp.segment<3>(index.acce_scale_index_) *= (2.0 / astime);
    }
    state_q_ = state_q_tmp.asDiagonal();

    std::string output_file_path = config_->get<std::string>("result_output_path");
    output_file_path.append("/navinfo.txt");
    ofs_result_output_.open(output_file_path);
    if (!ofs_result_output_.good())
    {
        LOG(ERROR) << "Open Result File Failed\t" << output_file_path << std::endl;
    }
    // ofs_result_output_ << nav_info_ << std::endl;
    nav_info_bak_ = nav_info_;
    return true;
}

void State::ReviseState(const Eigen::VectorXd &dx)
{
    filter_->ReviseState(nav_info_, dx);
}

/**
 * @brief  数据处理入口
 * @note   
 * @retval None
 */
void State::StartProcessing()
{
    using namespace camera;
    if (!InitializeState())
    {
        navexit();
    }
    static int state_count = filter_->GetStateSize();
    static int output_rate = config_->get<int>("result_output_rate");
    static auto &index = filter_->GetStateIndex();
    GnssData::Ptr ptr_gnss_data = nullptr;
    CameraData::Ptr ptr_camera_data = nullptr;
    ImuData::Ptr ptr_pre_imu_data = std::make_shared<ImuData>(), ptr_curr_imu_data;
    BaseData::bPtr base_data;
    Eigen::MatrixXd PHI = Eigen::MatrixXd::Identity(state_count, state_count);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_count);
    int data_rate = config_->get<int>("data_rate");
    LOG(INFO) << "Starting Processing Data...." << std::endl;
    while (true)
    {
        base_data = data_queue_->GetData();
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
            *ptr_pre_imu_data = *ptr_curr_imu_data;
            ptr_pre_imu_data->set_time(nav_info_.time_);
            if (!(ptr_curr_imu_data->get_time() - nav_info_.time_ <= 1.0 / data_rate))
            {
                LOG(ERROR) << "Data queue exists bug(s)" << std::endl;
                navsleep(1000);
            }
            break;
        }
    }
    bool gnss_update = false;
    while (true)
    {
        if (ptr_curr_imu_data != nullptr)
        {
            Eigen::MatrixXd phi;
            double dt = ptr_curr_imu_data->get_time() - ptr_pre_imu_data->get_time();
            (*ptr_curr_imu_data) = Compensate(*ptr_curr_imu_data, nav_info_, dt);
            nav_info_ = navmech::MechanicalArrangement(*ptr_pre_imu_data, *ptr_curr_imu_data, nav_info_, phi);
            PHI *= phi;
            ptr_pre_imu_data = ptr_curr_imu_data;
            ptr_curr_imu_data = nullptr;
        }
        else
        {
            /* time update */
            double dt = base_data->get_time() - latest_update_time_;
            auto &Rbe = nav_info_.rotation_;
            Eigen::MatrixXd state_q_used = state_q_;
            state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) =
                Rbe * state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) =
                Rbe * state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.att_index_, index.att_index_) =
                Rbe * state_q_used.block<3, 3>(index.att_index_, index.att_index_) * Rbe.transpose();
            Eigen::MatrixXd Q = (PHI * state_q_used * PHI.transpose() + state_q_used) * 0.5 * dt;
            filter_->TimeUpdate(PHI, Q, base_data->get_time());
            PHI = Eigen::MatrixXd::Identity(state_count, state_count);
            latest_update_time_ = base_data->get_time();

            /* measure update */
            if (ptr_gnss_data != nullptr)
            {
                gps_process_->processing(ptr_gnss_data, nav_info_, dx);
                ReviseState(dx);
                if (camera_enable)
                {
                    int cam_imu_idx = config_->get<int>("evaluate_camera_imu_rotation") == 0 ? 0 : 3;
                    msckf_process_->ReviseCameraState(dx.tail(dx.size() - filter_->GetStateIndex().total_state + cam_imu_idx));
                }
                ptr_gnss_data = nullptr;
                gnss_update = true;
            }
            else if (ptr_camera_data != nullptr)
            {
                msckf_process_->ProcessImage(ptr_camera_data->image_, ptr_camera_data->get_time(), nav_info_);
                ptr_camera_data = nullptr;
            }
        }

        int idt = int((nav_info_.time_.Second() + 0.00005) * output_rate) - int(nav_info_bak_.time_.Second() * output_rate);
        if (idt > 0)
        {
            double second_of_week = nav_info_.time_.SecondOfWeek();
            double double_part = (second_of_week * output_rate - int(second_of_week * output_rate)) / output_rate;
            NavTime inter_time = nav_info_.time_ - double_part;
            auto output_nav = InterpolateNavInfo(nav_info_bak_, nav_info_, inter_time);
            output_nav.pos_ = earth::CorrectLeverarmPos(output_nav);
            auto blh = earth::WGS84XYZ2BLH(output_nav.pos_);
            output_nav.vel_ = earth::CalCe2n(blh(0), blh(1)) * earth::CorrectLeverarmVel(output_nav);
            ofs_result_output_ << output_nav << std::endl;
            if (fabs(int(output_nav.time_.SecondOfWeek()) - output_nav.time_.SecondOfWeek() < 0.04))
                LOG(ERROR) << output_nav.time_ << std::endl;
            gnss_update = false;
        }
        nav_info_bak_ = nav_info_;
        /*获取新的数据 */
        base_data = data_queue_->GetData();
        // LOG(INFO) << "base_data time: " << base_data->get_type() << "  "
        //           << std::fixed << std::setprecision(8)
        //           << base_data->get_time().SecondOfWeek() << std::endl;
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
        }
        else if (base_data->get_type() == GNSSDATA)
        {
            ptr_gnss_data = std::dynamic_pointer_cast<GnssData>(base_data);
        }
        else if (base_data->get_type() == CAMERADATA)
        {
            ptr_camera_data = std::dynamic_pointer_cast<CameraData>(base_data);
        }
        else if (base_data->get_type() == DATAUNKOWN)
        {
            break;
        }
        else
        {
            LOG(ERROR) << "Bak Data processing thread " << std::endl;
            //BAK for other data type
        }
    }
    LOG(INFO) << "all data processing finish" << std::endl;
}

NavInfo State::GetNavInfo() const
{
    return nav_info_;
}

} // namespace mscnav
