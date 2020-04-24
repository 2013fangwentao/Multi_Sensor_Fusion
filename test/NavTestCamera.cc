/*
** NavTestCamera.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 22 下午7:22:24 2019 little fang
** Last update Wed Mar 10 上午11:32:45 2020 little fang
*/

#include "camera/data.hpp"
#include "navattitude.hpp"
#include "constant.hpp"
#include "navlog.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace utiltool;

// 89.4966 - 3.5274 87.8094

int main(int argc, char const *argv[])
{
    Eigen::Vector3d m_att(89.4966, -3.5274, 87.8094);
    m_att *= constant::deg2rad;
    std::cout << std::fixed << std::setprecision(8) << utiltool::attitude::Euler2RotationMatrix(m_att) << std::endl;
    Eigen::Matrix3d mat;
    mat << 0.03922729, 0.00696737, 0.99920602, 0.99650908, -0.07402227, -0.03860526, 0.07369453, 0.99723225, -0.00984674;
    auto att = utiltool::attitude::RotationMartix2Euler(mat);
    mat << 0.0107451, -0.00760956, 0.99991331, 0.99955609, -0.02770711, -0.01095212, 0.02778805, 0.99958712, 0.00730847;
    auto att2 = utiltool::attitude::RotationMartix2Euler(mat);
    std::cout << std::fixed << std::setprecision(4) << att.transpose() * constant::rad2deg << std::endl;
    std::cout << std::fixed << std::setprecision(4) << att2.transpose() * constant::rad2deg << std::endl;
    mat << 0.01106943, -0.02297247, 0.99967481, 0.99955543, -0.02742435, -0.01169831, 0.02768417, 0.99935988, 0.02265869;
    auto att3 = utiltool::attitude::RotationMartix2Euler(mat);
    std::cout << std::fixed << std::setprecision(4) << att3.transpose() * constant::rad2deg << std::endl;
    // if (argc != 2)
    // {
    //     std::cout << "executable image" << std::endl;
    //     return 0;
    // }
    // cv::Mat im = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    // cv::imshow("camera", im);
    // cv::waitKey(0);
    // utiltool::NavTime time = utiltool::NavTime::NowTime();
    // mscnav::camera::CameraData camera1;
    // {
    //     mscnav::camera::CameraData camera(time, im);
    //     camera1 = camera;
    //     cv::imshow("camera2", camera.image_);
    //     cv::waitKey(0);
    // }
    // cv::imshow("camera3", camera1.image_);
    // cv::waitKey(0);
    return 0;
}
