/*
** NavTestCamera.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 22 下午7:22:24 2019 little fang
** Last update Sat Aug 23 上午11:13:31 2019 little fang
*/

#include "camera/data.hpp"
#include "navlog.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cout << "executable image" << std::endl;
        return 0;
    }
    cv::Mat im = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("camera", im);
    cv::waitKey(0);
    utiltool::NavTime time = utiltool::NavTime::NowTime();
    mscnav::camera::CameraData camera1;
    {
        mscnav::camera::CameraData camera(time, im);
        camera1 = camera;
        cv::imshow("camera2", camera.image_);
        cv::waitKey(0);
    }
    cv::imshow("camera3", camera1.image_);
    cv::waitKey(0);
    return 0;
}
