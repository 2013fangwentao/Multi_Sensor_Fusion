/*
** ImageUndistort.cc for ImageUndistort in /media/fwt/Data/program/mscnav/tools
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Nov 5 上午10:09:45 2019 little fang
** Last update Wed Nov 5 下午2:22:03 2019 little fang
*/
#include "navconfig.hpp"
#include "navlog.hpp"
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace utiltool;

int main(int argc, const char **argv)
{
    LogInit(argv[0], ".");
    ConfigInfo::Ptr config = ConfigInfo::GetInstance();
    config->open(argv[1]);
    auto undistort_coeff = config->get_array<double>("camera_distcoeffs");
    auto camera_par = config->get_array<double>("camera_intrinsic");

    cv::Mat camera_mat_ = (cv::Mat_<double>(3, 3) << camera_par[0], 0.0, camera_par[2],
                           0.0, camera_par[1], camera_par[3],
                           0.0, 0.0, 1.0);

    cv::Mat dist_coeffs_ = (cv::Mat_<double>(5, 1) << undistort_coeff[0], undistort_coeff[1], undistort_coeff[2], undistort_coeff[3], undistort_coeff[4]);

    std::string image_config = config->get<std::string>("camera_config_file_path");
    std::ifstream ifs_image_config(image_config);
    if (!ifs_image_config.good())
    {
        LOG(FATAL) << "read file error!" << std::endl;
    }
    std::string image_path, tmp_line;
    std::getline(ifs_image_config, image_path);

    while (!ifs_image_config.eof())
    {
        std::getline(ifs_image_config, tmp_line);
        auto data = utiltool::TextSplit(tmp_line, "\\s+");
        if (data.size() < 3)
            continue;
        cv::Mat im = cv::imread((image_path + "/" + data[2]), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat dst;
        cv::undistort(im, dst, camera_mat_, dist_coeffs_);
        cv::imshow("原始图像", im);
        cv::imshow("矫正图像", dst);
        cv::waitKey(1);
        // LOG_EVERY_N(INFO, 10) << camera_mat_ << "\t" << dist_coeffs_ << std::endl;
    }
    return 0;
}