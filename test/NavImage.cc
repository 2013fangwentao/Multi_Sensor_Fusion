/*
** NavImage.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午3:21:09 2019 little fang
** Last update Thu Aug 8 下午3:21:09 2019 little fang
*/

#include "camera/imageprocess.h"
#include "navlog.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace mscnav::camera;

int main(int argc, char const *argv[])
{
    utiltool::LogInit(argv[0], ".");
    ImageProcess::Initialize(1200, 1.2, 8, 20, 7);
    cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    // cv::imshow("img1", img1);
    // cv::imshow("img2", img2);
    std::vector<cv::KeyPoint> key_point1, key_point2;
    cv::Mat descriptors1, descriptors2;
    ImageProcess::OrbFreatureExtract(img1, key_point1, descriptors1);
    ImageProcess::OrbFreatureExtract(img2, key_point2, descriptors2);
    cv::drawKeypoints(img1, key_point1, img1, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::drawKeypoints(img2, key_point2, img2, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("key_point1", img1);
    cv::imshow("key_point2", img2);

    cv::Mat image_matches;
    std::vector<cv::DMatch> matches;
    ImageProcess::FreatureMatch(key_point1, descriptors1, key_point2, descriptors2, matches);
    cv::drawMatches(img1, key_point1, img2, key_point2, matches, image_matches);
    cv::imshow("匹配的点集", image_matches);

    cv::waitKey(0);
    return 0;
}

