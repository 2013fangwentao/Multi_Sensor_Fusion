/*
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Nov 5 上午10:09:45 2019 little fang
** Last update Tue Feb 3 下午9:33:03 2020 little fang
*/
#include "navconfig.hpp"
#include "navlog.hpp"
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace utiltool;

std::vector<cv::KeyPoint> keypoints_vector;

int main(int argc, const char **argv)
{


    LogInit(argv[0], ".");

    cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    std::ifstream ifs_keypoints(argv[2]);
    int idx = 0;
    while (!ifs_keypoints.eof())
    {
        std::string line;
        std::getline(ifs_keypoints, line);
        auto dat = utiltool::TextSplit(line, "\\s+");
        cv::Point2f temp(stod(dat[0]), stod(dat[1]));
        cv::circle(img1, temp, 3, (0, 255, 0));
        cv::putText(img1, std::to_string(idx + 1), temp, cv::FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv::LINE_AA);
        idx++;
    }
    cv::imshow("key points", img1);
    cv::waitKey(0);

    return 0;
}