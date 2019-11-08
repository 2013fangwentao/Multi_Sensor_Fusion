/*
** navimu.h for mscnav in /media/fwt/Data/program/mscnav/include/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 上午10:01:57 2019 little fang
** Last update Thu Nov 6 下午8:04:27 2019 little fang
*/

#ifndef IMUDATA_H_
#define IMUDATA_H_

#include "stdio.h"
#include <thread>
#include <fstream>
#include <mutex>
#include <memory>
#include <atomic>

#include "navstruct.hpp"

namespace mscnav
{
using utiltool::ImuData;
using utiltool::IMUDATAPOOL;
const int MAX_SIZE_IMUPOOL(50);

class ImuDataCollect
{
public:
	using Ptr = std::shared_ptr<ImuDataCollect>;

public:
	ImuDataCollect();
	virtual ~ImuDataCollect();

public:
	virtual bool StartReadData() = 0;
	virtual bool GetImuData(ImuData::Ptr &currimudata) = 0;

protected:
	std::mutex mtx_imudata_;
	std::thread th_collectimudata_;
	std::atomic<int> int_markofcollectdata_; // -1 no start 0 collecting 1 end collect
	IMUDATAPOOL imudatapool_;
};

class FileImuData : public ImuDataCollect
{
public:
	FileImuData() {}
	~FileImuData() {}

public:
	bool GetImuData(ImuData::Ptr &currimudata) override;
	bool StartReadData() override;

private:
	void ReadingIMUData();

private:
	std::ifstream ifs_imufile_;
};
} // namespace mscnav

#endif