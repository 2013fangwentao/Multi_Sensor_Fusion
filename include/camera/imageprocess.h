/*
** imageprocess.h for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue Aug 6 下午5:13:31 2019 little fang
** Last update Sat Apr 24 下午3:24:39 2020 little fang
*/

#ifndef IMAGE_PROCESS_H_
#define IMAGE_PROCESS_H_

#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"

namespace mscnav
{
namespace camera
{

class ImageProcess
{
private:
    static cv::Ptr<cv::DescriptorMatcher> matcher_;
    static bool is_initialed_;
    static cv::Ptr<cv::ORB> cv_orb_;
    static cv::Ptr<cv::Feature2D> cv_detector_ptr;
    static cv::CascadeClassifier CarDetector;
    static cv::CascadeClassifier BodyDetector;

private:
    ImageProcess() {}
    ~ImageProcess() {}

public:
    static void Initialize(int nfeatures, float scale_factor, int nlevels,
                           int ini_th_fast, int min_th_fast);

    static void OrbFreatureExtract(const cv::InputArray &image,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::OutputArray descriptors);

    static void GoodFreatureDetect(const cv::Mat &image,
                                   std::vector<unsigned long long int> &keypoints_id,
                                   std::vector<cv::Point2f> &keypoints);

    static void PredictFeatureTracking(const std::vector<cv::Point2f> &input_pts,
                                       const cv::Matx33f &R_p_c,
                                       const cv::Matx33f &intrinsics,
                                       std::vector<cv::Point2f> &compensated_pts);

    static void CheckFeatureTrack(std::vector<cv::Point2f> &pre_keypoints,
                                  std::vector<cv::Point2f> &curr_keypoints,
                                  std::vector<unsigned long long int> &keypoints_id,
                                  const Eigen::Matrix3d &R_p_c,
                                  const Eigen::Vector3d &t_p_c,
                                  const cv::Mat &dist_coeffs,
                                  const cv::Mat &intrinsics);

    static void LKTrack(const cv::Mat &pre_image,
                        const cv::Mat &curr_image,
                        // const Eigen::Matrix3d &rotation,
                        std::vector<unsigned long long int> &keypoints_id,
                        std::vector<cv::Point2f> &pre_keypoints,
                        std::vector<cv::Point2f> &curr_keypoints);

    static void FreatureMatch(const std::vector<cv::KeyPoint> &keypoints1,
                              const cv::Mat &descriptors1,
                              const std::vector<cv::KeyPoint> &keypoints2,
                              const cv::Mat &descriptors2,
                              std::vector<cv::DMatch> &matches,
                              float default_min_distance = 40.0);

    static void TwoPointRansac(const std::vector<cv::Point2f> &pts1,
                               const std::vector<cv::Point2f> &pts2,
                               const cv::Matx33f &R_p_c,
                               const cv::Mat &intrinsics,
                               const cv::Mat &distortion_coeffs,
                               const double &inlier_error,
                               const double &success_probability,
                               std::vector<int> &inlier_markers);

private:
    static void OutlierRemove(const std::vector<cv::KeyPoint> &keypoints1,
                              const std::vector<cv::KeyPoint> &keypoints2,
                              std::vector<cv::DMatch> &matches);

    static void OutlierRemove(std::vector<cv::Point2f> &keypoints1,
                              std::vector<cv::Point2f> &keypoints2,
                              // const Eigen::Matrix3d &rotation,
                              std::vector<unsigned long long int> &keypoints_id);
    static void rescalePoints(std::vector<cv::Point2f> &pts1,
                              std::vector<cv::Point2f> &pts2,
                              float &scaling_factor);
};

} // namespace camera
} // namespace mscnav

#endif /* !IMAGE_PROCESS_H_ */
