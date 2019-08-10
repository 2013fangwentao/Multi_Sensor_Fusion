/*
** imageprocess.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 7 上午11:55:45 2019 little fang
** Last update Fri Aug 8 下午8:45:18 2019 little fang
*/

#include "camera/imageprocess.h"
#include "navlog.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace ORB_SLAM2;

namespace mscnav
{
namespace camera
{
bool ImageProcess::is_initialed_(false);
std::shared_ptr<ORBextractor> ImageProcess::orb_extractor_(nullptr);
cv::Ptr<cv::DescriptorMatcher> ImageProcess::matcher_(cv::DescriptorMatcher::create("BruteForce-Hamming"));

void ImageProcess::Initialize(int nfeatures, float scale_factor, int nlevels,
                              int ini_th_fast, int min_th_fast)
{
    if (is_initialed_)
        return;
    orb_extractor_ =
        std::make_shared<ORBextractor>(nfeatures,
                                       scale_factor,
                                       nlevels,
                                       ini_th_fast,
                                       min_th_fast);
    // matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    is_initialed_ = true;
}

/**
 * @brief  调用ORB-Slam2中现成的ORB特征点提取函数，获得特征点及其描述子
 * @note   
 * @param  image: 输入图像
 * @param  &keypoints: 提取的关键点，输出给用户
 * @param  descriptors: 对应关键点的描述子，输出给用户
 * @retval None
 */
void ImageProcess::OrbFreatureExtract(const cv::InputArray &image,
                                      std::vector<cv::KeyPoint> &keypoints,
                                      cv::OutputArray descriptors)
{
    if (!is_initialed_)
        LOG(ERROR) << " Image Process do not initialized !" << std::endl
                   << " Image Process do not initialized !" << std::endl
                   << " Image Process do not initialized !" << std::endl;
    (*orb_extractor_)(image, cv::Mat(), keypoints, descriptors);
    return;
}

/**
 * @brief  利用OPENCV的库实现特征点的匹配
 * @note   
 * @param  &descriptors1: 
 * @param  &descriptors2: 
 * @param  &matches: 匹配结果
 * @param  default_min_distance:  默认的Hamming距离的最小值，可不赋值
 * @retval None
 */
void ImageProcess::FreatureMatch(const std::vector<cv::KeyPoint> &keypoints1,
                                 const cv::Mat &descriptors1,
                                 const std::vector<cv::KeyPoint> &keypoints2,
                                 const cv::Mat &descriptors2,
                                 std::vector<cv::DMatch> &matches,
                                 float default_min_distance)
{
    if (!is_initialed_)
        LOG(ERROR) << " Image Process do not initialized !" << std::endl
                   << " Image Process do not initialized !" << std::endl
                   << " Image Process do not initialized !" << std::endl;
    matches.clear();
    std::vector<cv::DMatch> tmp_matches;
    matcher_->match(descriptors1, descriptors2, tmp_matches);
    double min_dist = min_element(tmp_matches.begin(), tmp_matches.end(),
                                  [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; })
                          ->distance;
    float threshold_distance = default_min_distance > 3 * min_dist ? default_min_distance : 3 * min_dist;

LEAP:
    for (auto element : tmp_matches)
    {
        if (element.distance < threshold_distance)
        {
            matches.push_back(element);
        }
    }
    if (matches.size() < 9)
    {
        threshold_distance *= 1.5;
        matches.clear();
        goto LEAP;
    }

    OutlierRemove(keypoints1, keypoints2, matches);
    LOG_EVERY_N(INFO, 10) << "matched point count is " << matches.size() << std::endl;
    return;
}

/**
 * @brief  利用RANSAC方法剔除外点，留下匹配度较好的点
 * @note   
 * @param  &keypoints1: 
 * @param  &keypoints2: 
 * @param  &matches: 
 * @retval None
 */
void ImageProcess::OutlierRemove(const std::vector<cv::KeyPoint> &keypoints1,
                                 const std::vector<cv::KeyPoint> &keypoints2,
                                 std::vector<cv::DMatch> &matches)
{
    std::vector<uchar> ransac_status;
    std::vector<cv::Point2f> keypointf1, keypointf2;
    for (auto element : matches)
    {
        auto &kp1 = keypoints1[element.queryIdx].pt;
        auto &kp2 = keypoints1[element.trainIdx].pt;
        keypointf1.push_back(kp1);
        keypointf2.push_back(kp2);
    }
    cv::Mat Homography = cv::findFundamentalMat(keypointf1, keypointf2, ransac_status, cv::FM_RANSAC, 10.0);
    LOG_IF(ERROR, (ransac_status.size() != matches.size())) << " ERROR ransac size != matches size" << std::endl;
    for (size_t i = 0; i < ransac_status.size(); i++)
    {
        if (ransac_status[i] == 0)
        {
            matches.at(i).distance = -999.0;
        }
    }
    for (auto iter = matches.begin(); iter != matches.end();)
    {
        if (iter->distance == -999.0)
        {
            matches.erase(iter);
        }
        else
        {
            iter++;
        }
    }
    return;
}
} // namespace camera

} // namespace mscnav
