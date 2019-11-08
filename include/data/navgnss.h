/*
** navgnss.h for mscnav in /media/fwt/Data/program/mscnav/include/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 上午10:02:18 2019 little fang
** Last update Thu Nov 6 下午8:04:40 2019 little fang
*/

#ifndef GNSSDATA_H_
#define GNSSDATA_H_

#include "stdio.h"
#include "math.h"
#include "navstruct.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>

namespace mscnav
{
const int MAX_SIZE_GNSSPOOL(50);

using utiltool::GnssData;
using utiltool::GNSSDATAPOOL;

void GetStartAndEndTime(utiltool::NavTime &start_time, utiltool::NavTime &end_time);

/**
 * @brief  base class for gps data,bak for realtime Gnss data collect
 * @note   
 * @retval None
 */
class GnssDataCollect
{
public:
	using Ptr = std::shared_ptr<GnssDataCollect>;

public:
	GnssDataCollect(bool logout) : aint_markofcollectdata_{-1}, logout_(logout) {}
	virtual ~GnssDataCollect();

public:
	virtual bool StartReadGnssData() = 0;		 //{ return false; };
	virtual bool GetData(GnssData::Ptr &gd) = 0; //{ return false; }

protected:
	GNSSDATAPOOL gd_datapool_;
	std::mutex mtx_collectdata_;
	std::thread th_collectgdata_;
	std::atomic<int> aint_markofcollectdata_;
	std::ofstream ofs_log_;
	bool logout_;
};

class FileGnssData : public GnssDataCollect
{
public:
	FileGnssData(bool logout = false) : GnssDataCollect(logout) {}
	~FileGnssData() {}

public:
	bool StartReadGnssData() override;
	bool GetData(GnssData::Ptr &gd) override;

private:
	void ReadingData();

private:
	std::ifstream ifs_file_;
};
} // namespace mscnav
#endif