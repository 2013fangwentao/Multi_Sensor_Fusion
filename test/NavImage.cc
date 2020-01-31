/*
** NavImage.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午3:21:09 2019 little fang
** Last update Sun Jan 11 下午3:08:35 2020 little fang
*/

#include "camera/imageprocess.h"
#include "navlog.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

using namespace mscnav::camera;

// int main(int argc, char const *argv[])
// {
//     utiltool::LogInit(argv[0], ".");
//     ImageProcess::Initialize(1200, 1.2, 8, 20, 7);
//     cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
//     cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
//     // cv::imshow("img1", img1);
//     // cv::imshow("img2", img2);
//     std::vector<cv::KeyPoint> key_point1, key_point2;
//     cv::Mat descriptors1, descriptors2;
//     ImageProcess::OrbFreatureExtract(img1, key_point1, descriptors1);
//     ImageProcess::OrbFreatureExtract(img2, key_point2, descriptors2);
//     cv::drawKeypoints(img1, key_point1, img1, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
//     cv::drawKeypoints(img2, key_point2, img2, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
//     cv::imshow("key_point1", img1);
//     cv::imshow("key_point2", img2);

//     cv::Mat image_matches;
//     std::vector<cv::DMatch> matches;
//     ImageProcess::FreatureMatch(key_point1, descriptors1, key_point2, descriptors2, matches);
//     cv::drawMatches(img1, key_point1, img2, key_point2, matches, image_matches);
//     cv::imshow("匹配的点集", image_matches);

//     cv::waitKey(0);
//     return 0;
// }

int main(int argc, char const *argv[])
{
    if (argc != 3)
    {
        return 0;
    }
    utiltool::LogInit(argv[0], ".", 0);
    ImageProcess::Initialize(800, 1.2, 8, 20, 7);

    std::string image_path = argv[1];
    std::string output_path = argv[2];
    std::ifstream ifs_name_deque(image_path + ("/name.txt"));
    cv::Mat pre_image;
    std::vector<cv::KeyPoint> pre_keypoint;
    cv::Mat pre_descr;
    std::vector<cv::DMatch> matches;
    while (!ifs_name_deque.eof())
    {
        std::vector<cv::KeyPoint> curr_keypoint;
        cv::Mat curr_descr;
        std::string line;
        std::getline(ifs_name_deque, line);
        cv::Mat curr_image = cv::imread(image_path + "/" + line + ".png", CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat curr_image_half = curr_image.rowRange(0, 320);
        ImageProcess::OrbFreatureExtract(curr_image_half, curr_keypoint, curr_descr);
        cv::Mat key_image;
        cv::drawKeypoints(curr_image, curr_keypoint, key_image, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("key point", key_image);
        cv::waitKey(1);

        cv::Mat image_matches;
        if (!pre_image.empty())
        {
            ImageProcess::FreatureMatch(pre_keypoint,
                                        pre_descr,
                                        curr_keypoint,
                                        curr_descr,
                                        matches,
                                        40.0);
            cv::drawMatches(pre_image, pre_keypoint, curr_image, curr_keypoint, matches, image_matches);
            // cv::imshow("匹配数据", image_matches);
            // cv::waitKey(1);
            cv::imwrite(output_path + "/match_" + line + ".png", image_matches);
        }
        pre_image = curr_image.clone();
        pre_descr = curr_descr.clone();
        pre_keypoint = curr_keypoint;
    }

    return 0;
}
