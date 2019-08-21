/*
** data.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 21 下午9:24:01 2019 little fang
** Last update Thu Aug 21 下午9:48:52 2019 little fang
*/

#ifndef CAMERA_DATA_H_
#define CAMERA_DATA_H_

#include "navstruct.hpp"
#include "navtime.h"
#include <opencv2/core/core.hpp>

namespace mscnav
{

namespace camera
{

using utiltool::BaseData;
using utiltool::NavTime;

class CameraData : public BaseData
{
public:
    using Ptr = std::shared_ptr<CameraData>;
    CameraData(const NavTime &time) : BaseData(time)
    {
        data_type_ = utiltool::DataType::CAMERADATA;
    };
    CameraData()
    {
        data_type_ = utiltool::DataType::CAMERADATA;
    }
    ~CameraData() {}
    CameraData &operator=(const CameraData &camera)
    {
        *this = camera;
        this->image = camera.image.clone();
        return *this;
    }
    CameraData(const CameraData &camera)
    {
        *this = camera;
        this->image = camera.image.clone();
    }

public:
    cv::Mat image;
};
using CAMERADATAPOOL = std::deque<CameraData>;

} // namespace camera
} // namespace mscnav

#endif /* !CAMERA_DATA_H_ */
