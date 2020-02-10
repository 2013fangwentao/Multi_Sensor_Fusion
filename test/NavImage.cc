/*
** NavImage.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Aug 8 下午3:21:09 2019 little fang
** Last update Sun Jan 11 下午3:08:35 2020 little fang
*/

// #include "camera/imageprocess.h"
// #include "navlog.hpp"
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <fstream>

// using namespace mscnav::camera;

// // int main(int argc, char const *argv[])
// // {
// //     utiltool::LogInit(argv[0], ".");
// //     ImageProcess::Initialize(1200, 1.2, 8, 20, 7);
// //     cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
// //     cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
// //     // cv::imshow("img1", img1);
// //     // cv::imshow("img2", img2);
// //     std::vector<cv::KeyPoint> key_point1, key_point2;
// //     cv::Mat descriptors1, descriptors2;
// //     ImageProcess::OrbFreatureExtract(img1, key_point1, descriptors1);
// //     ImageProcess::OrbFreatureExtract(img2, key_point2, descriptors2);
// //     cv::drawKeypoints(img1, key_point1, img1, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
// //     cv::drawKeypoints(img2, key_point2, img2, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
// //     cv::imshow("key_point1", img1);
// //     cv::imshow("key_point2", img2);

// //     cv::Mat image_matches;
// //     std::vector<cv::DMatch> matches;
// //     ImageProcess::FreatureMatch(key_point1, descriptors1, key_point2, descriptors2, matches);
// //     cv::drawMatches(img1, key_point1, img2, key_point2, matches, image_matches);
// //     cv::imshow("匹配的点集", image_matches);

// //     cv::waitKey(0);
// //     return 0;
// // }

// int main(int argc, char const *argv[])
// {
//     if (argc != 3)
//     {
//         return 0;
//     }
//     utiltool::LogInit(argv[0], ".", 0);
//     ImageProcess::Initialize(800, 1.2, 8, 20, 7);

//     std::string image_path = argv[1];
//     std::string output_path = argv[2];
//     std::ifstream ifs_name_deque(image_path + ("/name.txt"));
//     cv::Mat pre_image;
//     std::vector<cv::KeyPoint> pre_keypoint;
//     cv::Mat pre_descr;
//     std::vector<cv::DMatch> matches;
//     while (!ifs_name_deque.eof())
//     {
//         std::vector<cv::KeyPoint> curr_keypoint;
//         cv::Mat curr_descr;
//         std::string line;
//         std::getline(ifs_name_deque, line);
//         cv::Mat curr_image = cv::imread(image_path + "/" + line + ".png", CV_LOAD_IMAGE_GRAYSCALE);
//         cv::Mat curr_image_half = curr_image.rowRange(0, 320);
//         ImageProcess::OrbFreatureExtract(curr_image_half, curr_keypoint, curr_descr);
//         cv::Mat key_image;
//         cv::drawKeypoints(curr_image, curr_keypoint, key_image, cv::Scalar_<double>::all(-1), cv::DrawMatchesFlags::DEFAULT);
//         cv::imshow("key point", key_image);
//         cv::waitKey(1);

//         cv::Mat image_matches;
//         if (!pre_image.empty())
//         {
//             ImageProcess::FreatureMatch(pre_keypoint,
//                                         pre_descr,
//                                         curr_keypoint,
//                                         curr_descr,
//                                         matches,
//                                         40.0);
//             cv::drawMatches(pre_image, pre_keypoint, curr_image, curr_keypoint, matches, image_matches);
//             // cv::imshow("匹配数据", image_matches);
//             // cv::waitKey(1);
//             cv::imwrite(output_path + "/match_" + line + ".png", image_matches);
//         }
//         pre_image = curr_image.clone();
//         pre_descr = curr_descr.clone();
//         pre_keypoint = curr_keypoint;
//     }

//     return 0;
// }

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;
using namespace cv;
int main(int argc, char *argv[])
{

    cv::Mat output;
    cv::Mat gray;                       // current gray-level image
    cv::Mat gray_prev;                  // previous gray-level image
    std::vector<cv::Point2f> points[2]; // tracked features from 0->1
    std::vector<cv::Point2f> initial;   // initial position of tracked points

    std::vector<uchar> status; // status of tracked features
    std::vector<float> err;    // error in tracking

    cv::VideoCapture capture("bike.avi");
    if (!capture.isOpened())
    {
        return 0;
    }

    Mat frame;

    while (1)
    {
        capture >> frame;

        // convert to gray-level image
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        frame.copyTo(output);

        // 1. detect the points
        std::vector<cv::Point2f> features; // detected features
        int max_count = 500;               // maximum number of features to detect
        double qlevel = 0.01;              // quality level for feature detection
        double minDist = 10.0;             // minimum distance between two feature points

        if (points[0].size() <= 10)
        {
            cv::goodFeaturesToTrack(gray,      // the image
                                    features,  // the output detected features
                                    max_count, // the maximum number of features
                                    qlevel,    // quality level
                                    minDist);  // min distance between two features

            // add the detected features to the currently tracked features
            points[0].insert(points[0].end(), features.begin(), features.end());
            initial.insert(initial.end(), features.begin(), features.end());
        }
        // for first image of the sequence
        if (gray_prev.empty())
            gray.copyTo(gray_prev);

        // 2. track features
        TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
        double derivlambda = 0.5;
        int flags = 0;
        cv::calcOpticalFlowPyrLK(gray_prev, gray, // 2 consecutive images
                                 points[0],       // input point position in first image
                                 points[1],       // output point postion in the second image
                                 status,          // tracking success
                                 err,             // tracking error
                                 Size(31, 31), 3, criteria, derivlambda, flags);

        // 3. loop over the tracked points to reject the undesirables
        int k = 0;
        for (int i = 0; i < points[1].size(); i++)
        {
            // do we keep this point?
            if (status[i] && (abs(points[0][i].x - points[1][i].x) + (abs(points[0][i].y - points[1][i].y)) > 2))
            {
                // keep this point in vector
                initial[k] = initial[i];
                points[1][k++] = points[1][i];
            }
        }
        // eliminate unsuccesful points
        points[1].resize(k);
        initial.resize(k);

        // 4. draw all tracked points
        RNG rng;
        for (int i = 0; i < points[1].size(); i++)
        {
            // draw line and circle
            cv::line(output, initial[i], points[1][i], cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2, 8, 0);
            cv::circle(output, points[1][i], 3, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1);
        }

        // 5. current points and image become previous ones
        std::swap(points[1], points[0]);
        cv::swap(gray_prev, gray);

        cv::imshow("video_processing", output);
        if (cv::waitKey(80) >= 0)
        {
            break;
        }
    }

    cv::waitKey();
    std::cin.get();
    return 0;
}
