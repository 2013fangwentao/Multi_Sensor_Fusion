/*
** navcamera.h for mscnav in /media/fwt/Data/program/mscnav/include/data
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 21 下午9:36:31 2019 little fang
** Last update Fri Nov 7 上午11:09:49 2019 little fang
*/

#ifndef DATA_CAMERA_H_
#define DATA_CAMERA_H_

#include "camera/data.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <fstream>
#include <vector>

namespace mscnav
{
const int MAX_SIZE_CAMERAPOOL = 50;

class CameraDataCollect
{
public:
    using Ptr = std::shared_ptr<CameraDataCollect>;

public:
    CameraDataCollect(bool logout) : aint_markofcollectdata_{-1}, logout_(logout) {}
    virtual ~CameraDataCollect(){}

public:
    virtual bool StartReadCameraData() = 0;                //{ return false; };
    virtual bool GetData(camera::CameraData::Ptr &cd) = 0; //{ return false; };

protected:
    camera::CAMERADATAPOOL cd_datapool_;
    std::mutex mtx_collectdata_;
    std::thread th_collectgdata_;
    std::atomic<int> aint_markofcollectdata_;
    std::string log_path;
    bool logout_;
};

class FileCameraData : public CameraDataCollect
{
public:
    FileCameraData(bool logout = false) : CameraDataCollect(logout) {}
    ~FileCameraData();

    bool GetData(camera::CameraData::Ptr &cd) override;
    bool StartReadCameraData() override;
    
private:
    void ReadingData();    

private:
    std::ifstream ifs_camera_data_config_;
};
} // namespace mscnav
#endif /* !DATA_CAMERA_H_ */
