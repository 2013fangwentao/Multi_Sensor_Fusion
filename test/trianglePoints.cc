/*
** trianglePoints.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Feb 3 下午5:00:05 2020 little fang
** Last update Tue Feb 3 下午5:06:29 2020 little fang
*/

#include "navattitude.hpp"
#include "camera/msckf.hpp"
#include "camera/imageprocess.h"
#include "navbase.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char const *argv[])
{
    cv::Mat pts_4d;

    cv::Mat T2 = (cv::Mat_<double>(3, 4) << 0.9999988713756648, 0.00113306115553468, 0.00098662040035764, 0.03179405891569331,
                  -0.001131977968852206, 0.9999987567923283, -0.001097742991824135, 0.1007878631353378,
                  -0.0009878629837263564, 0.001096624920327915, 0.9999989107696617, 2.58494090847671);

    cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    std::vector<cv::Point2f> pts1, pts2;
    std::cout << T1 << std::endl;
    std::cout << T2 << std::endl;
    cv::Point2f p1(0.007889078, -0.13835451);
    pts1.push_back(p1);
    std::cout << p1 << std::endl;
    cv::Point2f p2(0.0080164066, -0.1559211);
    pts2.push_back(p2);
    std::cout << p2 << std::endl;

    cv::triangulatePoints(T1, T2, pts1, pts2, pts_4d);
    std::cout << pts_4d << std::endl;

    return 0;
}
