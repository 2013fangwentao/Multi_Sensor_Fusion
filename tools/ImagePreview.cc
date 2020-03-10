/*
** ImagePreview.cc for mscnav in /media/fwt/Data/program/mscnav/tools
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Mar 10 下午1:56:09 2020 little fang
** Last update Wed Mar 10 下午2:21:34 2020 little fang
*/

#include "data/navcamera.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include "navconfig.hpp"
using namespace cv;
using namespace std;
using namespace utiltool;

std::ifstream ifs_camera_data_config_;

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        std::cout << "usage: [ImagePreview] [ImagePathFiles]" << std::endl;
        return 0;
    }
    ifs_camera_data_config_.open(argv[1]);
    if (!ifs_camera_data_config_.good())
    {
        std::cout << "Read camera data config file failed" << std::endl;
        return false;
    }
    std::string image_path, tmp_line;
    getline(ifs_camera_data_config_, image_path);
    while (!ifs_camera_data_config_.eof())
    {
        std::getline(ifs_camera_data_config_, tmp_line);
        auto data = utiltool::TextSplit(tmp_line, "\\s+");
        if (data.size() < 3)
            continue;
        utiltool::NavTime time(std::stoi(data[0]), std::stod(data[1]));
        Mat im = imread((image_path + "/" + data[2]), CV_LOAD_IMAGE_GRAYSCALE);
        putText(im, data[1].c_str(), Point(40, 40), FONT_HERSHEY_TRIPLEX, 0.5, Scalar::all(-1));
        imshow("image", im);
        waitKey(5);
    }
    return 0;
}
