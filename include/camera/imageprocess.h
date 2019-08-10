/*
** imageprocess.h for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Aug 6 下午5:13:31 2019 little fang
** Last update Fri Aug 8 下午8:45:35 2019 little fang
*/

#ifndef IMAGE_PROCESS_H_
#define IMAGE_PROCESS_H_

#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include "ORBextractor.h"

namespace mscnav
{
namespace camera
{
using ORB_SLAM2::ORBextractor;

class ImageProcess
{
private:
    static std::shared_ptr<ORBextractor> orb_extractor_;
    static cv::Ptr<cv::DescriptorMatcher> matcher_;
    static bool is_initialed_;

private:
    ImageProcess() {}
    ~ImageProcess() {}

public:
    static void Initialize(int nfeatures, float scale_factor, int nlevels,
                           int ini_th_fast, int min_th_fast);

    static void OrbFreatureExtract(const cv::InputArray &image,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::OutputArray descriptors);

    static void FreatureMatch(const std::vector<cv::KeyPoint> &keypoints1,
                              const cv::Mat &descriptors1,
                              const std::vector<cv::KeyPoint> &keypoints2,
                              const cv::Mat &descriptors2,
                              std::vector<cv::DMatch> &matches,
                              float default_min_distance = 40.0);

private:
    static void OutlierRemove(const std::vector<cv::KeyPoint> &keypoints1,
                              const std::vector<cv::KeyPoint> &keypoints2,
                              std::vector<cv::DMatch> &matches);
};

} // namespace camera
} // namespace mscnav

#endif /* !IMAGE_PROCESS_H_ */
