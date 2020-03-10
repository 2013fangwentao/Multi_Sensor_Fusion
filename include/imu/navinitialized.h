/*
** navinitialized.h for mscnav in /media/fwt/Data/program/mscnav/include/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  undefined Jul 21 下午8:26:08 2019 little fang
** Last update undefined Jul 21 下午8:26:08 2019 little fang
*/

#ifndef IMU_INITIALIZED_H_
#define IMU_INITIALIZED_H_

#include "data/navdataque.h"
#include "navstruct.hpp"
#include "navconfig.hpp"
#include <algorithm>

namespace mscnav
{

enum AlignedMode
{
    UNKOWN = -1,
    SETTING = 0,               //配置文件直接设定
    STATIONARY_AND_MOTION = 1, //静止启动到速度大于某一个值
    IN_MOTION = 2              //起始时刻速度大于特定值,直接动对准
};
/**
 * @brief  只考虑低成本IMU的对准问题,加速度计输出计算俯仰横滚角,
 *         GPS速度对准航向,不考虑高等级设备的静态对准过程
 * @note   
 * @retval None
 */
class InitializedNav
{
public:
    InitializedNav(const DataQueue::Ptr &ptr_data_queue) : ptr_data_queue_(ptr_data_queue)
    {
        config = utiltool::ConfigInfo::GetInstance();
    }
    ~InitializedNav() {}
    using Ptr = std::shared_ptr<InitializedNav>;

public:
    bool StartAligning(utiltool::NavInfo &nav_info);
    Eigen::VectorXd &SetInitialVariance(Eigen::VectorXd &PVariance,
                                        utiltool::NavInfo &nav_info,
                                        const utiltool::StateIndex &index); 
    void SetStateIndex(utiltool::StateIndex &state_index);

private:
    AlignedMode aligned_mode_ = UNKOWN;
    utiltool::ConfigInfo::Ptr config;
    DataQueue::Ptr ptr_data_queue_;
   // utiltool::NavInfo n_info;
};
} // namespace mscnav

#endif /* !IMU_INITIALIZED_H_ */
