#include <iostream>
#include <chrono>
#include <exception>
#include "data/navgnss.h"
#include "navtime.h"
#include "navconfig.hpp"
#include "navlog.hpp"
#include "navbase.hpp"

namespace mscnav
{
using namespace utiltool;
using Eigen::Vector3d;
using std::chrono::milliseconds;

void GetStartAndEndTime(utiltool::NavTime &start_time, utiltool::NavTime &end_time)
{
	ConfigInfo::Ptr config = ConfigInfo::GetInstance();
	std::vector<int> utime = config->get_array<int>("process_date");
	double start_second = config->get<double>("start_time");
	double end_second = config->get<double>("end_time");
	NavTime start_time_tmp(utime[0], utime[1], utime[2], 0, 0, 0.0);
	NavTime end_time_tmp(utime[0], utime[1], utime[2], 0, 0, 0.0);
	start_time = start_time_tmp + int(start_second) % NavTime::MAXSECONDOFDAY + start_second - int(start_second);
	end_time = end_time_tmp + int(end_second) % NavTime::MAXSECONDOFDAY + end_second - int(end_second);
}

GnssDataCollect::~GnssDataCollect()
{
	if (logout_)
		ofs_log_.close();
	if (th_collectgdata_.joinable())
	{
		th_collectgdata_.join();
	}
}

/**
 * @brief  开启gnss数据线程, 读取Gnss数据
 * @note   
 * @retval 
 */
bool FileGnssData::StartReadGnssData()
{
	ConfigInfo::Ptr config = ConfigInfo::GetInstance();
	std::string gnss_data_path = config->get<std::string>("gnss_data_path");
	ifs_file_.open(gnss_data_path);
	if (!ifs_file_.good())
	{
		LOG(ERROR) << "Gnss Data File Open Failed! File path: " << gnss_data_path << std::endl;
		return false;
	}
	if (logout_)
	{
		std::string log_path = config->get<std::string>("result_output_path");
		NavTime time = NavTime::NowTime();
		log_path += "/gnss-data-" + time.Time2String() + ".log";
		ofs_log_.open(log_path);
	}
	bool break_time_type = (config->get<int>("gnss_break_type") == 0 ? true : false);
	if (break_time_type)
	{
		auto break_time = config->get_array<double>("gnss_break");
		DetermineBreakTime(break_time);
	}
	else
	{
		auto break_time = config->get_array<double>("gnss_break_array");
		DetermineBreakTimeArray(break_time);
	}
	aint_markofcollectdata_ = 0;
	std::thread th(&FileGnssData::ReadingData, this);
	th_collectgdata_ = std::move(th);
	return true;
}
/**
 * @brief  get the latest gnss data
 * @note   
 * @param  &gd: 
 * @retval 
 */
bool FileGnssData::GetData(GnssData::Ptr &gd)
{
	if (gd_datapool_.size() > 0)
	{
		std::unique_lock<std::mutex> lck(mtx_collectdata_);
		gd = gd_datapool_.front();
		gd_datapool_.pop_front();
		return true;
	}
	else if (aint_markofcollectdata_ == 0)
	{
		while (gd_datapool_.size() == 0)
		{
			LOG_EVERY_N(INFO, 200) << "GnssData thread sleep 10s" << std::endl;
			if (aint_markofcollectdata_ == 1)
			{
				return false;
			}
			std::this_thread::sleep_for(milliseconds(50));
		}
		std::unique_lock<std::mutex> lck(mtx_collectdata_);
		gd = gd_datapool_.front();
		gd_datapool_.pop_front();
		return true;
	}
	else
	{
		return false;
	}
}

void FileGnssData::ReadingData()
{
	// 获取计算的起止时间
	utiltool::NavTime end_time, start_time;
	GetStartAndEndTime(start_time, end_time);
	if (end_time <= start_time)
	{
		LOG(INFO) << "start time is " << start_time << std::endl;
		LOG(INFO) << "end time is " << end_time << std::endl;
		LOG(FATAL) << "start time is later than end time" << std::endl;
		return;
	}
	std::string linestr;
	while (!ifs_file_.eof())
	{
		getline(ifs_file_, linestr);
		if (linestr.size() == 0)
			continue;
		auto dat = TextSplit(linestr, "\\s+");
		if (dat.size() < 9)
		{
			LOG(ERROR) << "Data Format Error! \"" << linestr << "\"" << std::endl;
			continue;
		}
		try
		{
			NavTime time(std::stoi(dat[0]), std::stod(dat[1]));
			if (time < start_time || time > end_time)
				continue;
			GnssData::Ptr gd_temp = std::make_shared<GnssData>();
			gd_temp->set_time(time);
			gd_temp->pos_ = Vector3d{std::stod(dat[2]), std::stod(dat[3]), std::stod(dat[4])};
			gd_temp->pos_std_ = Vector3d{std::stod(dat[5]) , std::stod(dat[6]) , std::stod(dat[7]) };
			if (std::stoi(dat[8]) == 1 && (!IsBreak(time.SecondOfWeek())))
			{
				if (logout_)
				{
					ofs_log_ << gd_temp << std::endl;
				}
				std::unique_lock<std::mutex> lck(mtx_collectdata_);
				gd_datapool_.emplace_back(gd_temp);
			}
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
			LOG(ERROR) << e.what() << std::endl;
		}
		while (gd_datapool_.size() > MAX_SIZE_GNSSPOOL)
		{
			std::this_thread::sleep_for(milliseconds(200));
		}
	}
	aint_markofcollectdata_ = 1;
	ifs_file_.close();
}

void FileGnssData::DetermineBreakTimeArray(const std::vector<double> &break_time)
{
	bound_count_ = break_time.size() / 2;
	bound_count_ = bound_count_ > 20 ? 20 : bound_count_;
	memset(low_bound_, 0x0, sizeof(double) * 20);
	memset(up_bound_, 0x0, sizeof(double) * 20);
	for (int i = 0; i < bound_count_; i++)
	{
		low_bound_[i] = break_time.at(2 * i + 0);
		up_bound_[i] = break_time.at(2 * i + 1);
	}
}

void FileGnssData::DetermineBreakTime(const std::vector<double> &break_time)
{
	memset(low_bound_, 0x0, sizeof(double) * 20);
	memset(up_bound_, 0x0, sizeof(double) * 20);
	double breakstart = break_time.at(0);
	double breaktime = break_time.at(1);
	double breakinterval = break_time.at(2);
	bound_count_ = int(break_time.at(3));
	for (int i = 0; i < bound_count_; i++)
	{
		low_bound_[i] = breakstart + (breakinterval + breaktime) * i;
		up_bound_[i] = breakstart + breaktime + (breakinterval + breaktime) * i;
		LOG(ERROR) << "break time: " << low_bound_[i] << "," << up_bound_[i] << std::endl;
	}
}

bool FileGnssData::IsBreak(double sow)
{
	if (bound_count_ < 1)
	{
		return false;
	}
	for (int i = 0; i < bound_count_; i++)
	{
		if (sow > low_bound_[i] && sow < up_bound_[i])
		{
			return true;
		}
	}
	return false;
}

} // namespace mscnav
