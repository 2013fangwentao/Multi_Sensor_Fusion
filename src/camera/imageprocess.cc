/*
** imageprocess.cc for mscnav in /media/fwt/Data/program/mscnav/src/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 7 上午11:55:45 2019 little fang
** Last update Sat Apr 24 下午3:25:12 2020 little fang
*/

#include "camera/imageprocess.h"
#include "navlog.hpp"
#include "navbase.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/eigen.hpp>
#include "Eigen/Dense"
#include <numeric>


static unsigned long long int PointID = 0;

namespace mscnav
{
namespace camera
{
bool ImageProcess::is_initialed_(false);
cv::Ptr<cv::DescriptorMatcher> ImageProcess::matcher_(cv::DescriptorMatcher::create("BruteForce-Hamming"));
cv::Ptr<cv::ORB> ImageProcess::cv_orb_(cv::ORB::create(200));
cv::Ptr<cv::Feature2D> ImageProcess::cv_detector_ptr(cv::FastFeatureDetector::create(20));
cv::CascadeClassifier ImageProcess::CarDetector("sideview_cascade_classifier.xml");
cv::CascadeClassifier ImageProcess::BodyDetector("haarcascade_fullbody.xml");

void ImageProcess::Initialize(int nfeatures, float scale_factor, int nlevels,
                              int ini_th_fast, int min_th_fast)
{
    if (is_initialed_)
        return;
    // orb_extractor_ =
    //     std::make_shared<ORBextractor>(nfeatures,
    //                                    scale_factor,
    //                                    nlevels,
    //                                    ini_th_fast,
    //                                    min_th_fast);
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
    // (*orb_extractor_)(image, cv::Mat(), keypoints, descriptors);
    // cv_orb_->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    return;
}

static bool compare_keypoint(cv::KeyPoint p1, cv::KeyPoint p2)
{
    return p1.response > p2.response;
}

void ImageProcess::CheckFeatureTrack(std::vector<cv::Point2f> &pre_keypoints,
                                     std::vector<cv::Point2f> &curr_keypoints,
                                     std::vector<unsigned long long int> &keypoints_id,
                                     const Eigen::Matrix3d &R_p_c,
                                     const Eigen::Vector3d &t_p_c,
                                     const cv::Mat &dist_coeffs,
                                     const cv::Mat &intrinsics)
{
    std::vector<int> status(pre_keypoints.size());
    std::fill(status.begin(), status.end(), 1);
    std::vector<cv::Point2f> pre_keypoints_undistorted, curr_keypoints_undistorted;
    cv::undistortPoints(pre_keypoints, pre_keypoints_undistorted,
                        intrinsics, dist_coeffs, cv::noArray(), intrinsics);
    cv::undistortPoints(curr_keypoints, curr_keypoints_undistorted,
                        intrinsics, dist_coeffs, cv::noArray(), intrinsics);
    Eigen::Matrix3d K;
    cv::cv2eigen(intrinsics, K);
    K = K.inverse();

    Eigen::Matrix3d t_p_c_skew = utiltool::skew(t_p_c);
    for (size_t i = 0; i < pre_keypoints.size(); i++)
    {
        Eigen::Vector3d p1(pre_keypoints_undistorted[i].x, pre_keypoints_undistorted[i].y, 1.0);
        Eigen::Vector3d p2(curr_keypoints_undistorted[i].x, curr_keypoints_undistorted[i].y, 1.0);
        Eigen::MatrixXd coeff = p2.transpose() * K.transpose() * t_p_c_skew * R_p_c * K * p1;
        double threshold = coeff(0, 0) * 10e4;
        if (fabs(threshold) > 5)
        {
            status[i] = 0;
        }
    }
    int k = 0;
    for (int i = 0; i < curr_keypoints.size(); i++)
    {
        if (status[i] == 1)
        {
            keypoints_id[k] = keypoints_id[i];
            curr_keypoints[k] = curr_keypoints[i];
            pre_keypoints[k] = pre_keypoints[i];
            k++;
        }
    }
    keypoints_id.resize(k);
    curr_keypoints.resize(k);
    pre_keypoints.resize(k);
    return;
}

/**
 * @brief 根据单应性原理：已知一个平面的关键点可以得到另一个平面的关键点
 * @param input_pts：上一时刻的第一个相机对应的关键点
 * @param R_p_c: 利用imu数据计算得到的前后两个时刻图像帧的旋转初值
 * @param intrinsics:相机内参
 * @return compensated_pts:根据上一帧图像中的关键点位置预测得到当前帧的关键点位置
 *
 */
void ImageProcess::PredictFeatureTracking(
    const std::vector<cv::Point2f> &input_pts,
    const cv::Matx33f &R_p_c,
    const cv::Matx33f &intrinsics,
    std::vector<cv::Point2f> &compensated_pts)
{

    // Return directly if there are no input features.
    if (input_pts.size() == 0)
    {
        compensated_pts.clear();
        return;
    }
    compensated_pts.resize(input_pts.size());

    // Intrinsic matrix.
    // 相机内参矩阵K
    // cv::Matx33f K(
    //     intrinsics[0], 0.0, intrinsics[2],
    //     0.0, intrinsics[1], intrinsics[3],
    //     0.0, 0.0, 1.0);

    // 单应性矩阵的计算，公式推到：
    // x1 = K * X1_c; x2 = K * X2_c
    // x2_c = H * x1_c; X1_C = R_2_1 * X2_c
    // x1 = K * R_1_2 * K^inv * x2 --> H = K * R_1_2 * K^inv
    // TODO:这里应该还有平移向量，预测一个值所以可以去掉？
    cv::Matx33f H = intrinsics * R_p_c * intrinsics.inv();

    for (int i = 0; i < input_pts.size(); ++i)
    {
        cv::Vec3f p1(input_pts[i].x, input_pts[i].y, 1.0f);
        // 两个平面中的匹配点满足: x2 = H * x1
        // 归一化后的齐次坐标即另一个平面上匹配点的图像坐标
        cv::Vec3f p2 = H * p1;
        compensated_pts[i].x = p2[0] / p2[2];
        compensated_pts[i].y = p2[1] / p2[2];
    }

    return;
}

/**
 * @brief  调用opencv的goodFeaturesToTrack提取角点
 * @note   
 * @param  &image: 
 * @param  &keypoints: 
 * @retval None
 */
void ImageProcess::GoodFreatureDetect(const cv::Mat &image,
                                      std::vector<unsigned long long int> &keypoints_id,
                                      std::vector<cv::Point2f> &keypoints)
{
    static int grid_cols = 6, grid_rows = 6, min_num_points = 15, max_num_points = 20;
    static int grid_height = static_cast<int>(image.rows / grid_rows) + 1;
    static int grid_width = static_cast<int>(image.cols / grid_cols) + 1;

    // std::vector<std::vector<cv::Point2f *>> keypoints_tmp(grid_cols * grid_rows);
    std::vector<int> keypoints_tmp(grid_cols * grid_rows);
    std::vector<cv::Point2f> keypoints_new;

    for (auto iter : keypoints)
    {
        int row = static_cast<int>(iter.y / grid_height);
        int col = static_cast<int>(iter.x / grid_width);
        int code = row * grid_cols + col;
        // keypoints_tmp.at(code).emplace_back(&iter);
        keypoints_tmp.at(code) += 1;
    }

    cv::Mat mask(image.rows, image.cols, CV_8U, cv::Scalar(1));

    // // cv::Mat img = image.clone();
    // std::vector<cv::Rect> objects;
    // CarDetector.detectMultiScale(image, objects);
    // for (int i = 0; i < objects.size(); i++)
    // {
    //     mask(objects[i]) = 0;
    // }
    // objects.clear();
    // BodyDetector.detectMultiScale(image, objects);
    // for (int i = 0; i < objects.size(); i++)
    // {
    //     mask(objects[i]) = 0;
    // }

    for (auto &point : keypoints)
    {
        const int y = static_cast<int>(point.y);
        const int x = static_cast<int>(point.x);

        int up_lim = y - 10, bottom_lim = y + 10,
            left_lim = x - 10, right_lim = x + 10;
        if (up_lim < 0)
            up_lim = 0;
        if (bottom_lim > image.rows)
            bottom_lim = image.rows;
        if (left_lim < 0)
            left_lim = 0;
        if (right_lim > image.cols)
            right_lim = image.cols;
        cv::Range row_range(up_lim, bottom_lim);
        cv::Range col_range(left_lim, right_lim);
        mask(row_range, col_range) = 0;
    }
    for (size_t i = 0; i < grid_rows; i++)
    {
        for (size_t j = 0; j < grid_cols; j++)
        {
            if (keypoints_tmp.at(i * grid_rows + j) > min_num_points)
            {
                continue;
            }
            else
            {
                cv::Mat mask_tmp = mask.clone();
                if (i * grid_height != 0)
                {
                    cv::Range row_range(0, i * grid_height);
                    cv::Range col_range(0, image.cols);
                    mask_tmp(row_range, col_range) = 0;
                }
                if ((i + 1) * grid_height < image.rows)
                {
                    cv::Range row_range((i + 1) * grid_height, image.rows);
                    cv::Range col_range(0, image.cols);
                    mask_tmp(row_range, col_range) = 0;
                }
                if (j * grid_width != 0)
                {
                    cv::Range row_range(0, image.rows);
                    cv::Range col_range(0, j * grid_width);
                    mask_tmp(row_range, col_range) = 0;
                }
                if ((j + 1) * grid_width < image.cols)
                {
                    cv::Range row_range(0, image.rows);
                    cv::Range col_range((j + 1) * grid_width, image.cols);
                    mask_tmp(row_range, col_range) = 0;
                }
                std::vector<cv::KeyPoint> kkeypoints_new;
                cv_detector_ptr->detect(image, kkeypoints_new, mask_tmp);
                std::sort(kkeypoints_new.begin(), kkeypoints_new.end(), compare_keypoint);
                int rest_size = max_num_points - keypoints_tmp.at(i * grid_rows + j);
                for (size_t kk = 0; kk < kkeypoints_new.size() && kk < rest_size; kk++)
                {
                    keypoints_id.emplace_back(PointID++);
                    keypoints.emplace_back(kkeypoints_new.at(kk).pt);
                    keypoints_tmp.at(i * grid_rows + j) += 1;
                }
            }
        }
    }

    // cv::Range row_range(220, image.rows);
    // cv::Range col_range(0, image.cols);
    // mask(row_range, col_range) = 0;
    // std::vector<cv::KeyPoint> kkeypoints_new;
    // cv::Mat descriptors;
    // (*orb_extractor_)(image, mask, kkeypoints_new, descriptors);
    // cv_orb_->detect(image, kkeypoints_new, mask);
    // cv::goodFeaturesToTrack(image, keypoints_new, 200, 0.01, 5, mask, 5);
    // for (auto iter_tmp : kkeypoints_new)
    // // for (auto iter : keypoints_new)
    // {
    //     auto &iter = iter_tmp.pt;
    //     int row = static_cast<int>(iter.y / grid_height);
    //     int col = static_cast<int>(iter.x / grid_width);
    //     int code = row * grid_cols + col;
    //     if (keypoints_tmp.at(code).size() > max_num_points)
    //     {
    //         continue;
    //     }
    //     keypoints_id.emplace_back(PointID++);
    //     keypoints.emplace_back(iter);
    //     keypoints_tmp.at(code).emplace_back(&iter);
    // }
    return;
}

/**
 * @brief  
 * @note   
 * @param  &pre_image: 
 * @param  &curr_image: 
 * @param  &pre_keypoints: 
 * @param  &curr_keypoints: 
 * @retval None
 */
void ImageProcess::LKTrack(const cv::Mat &pre_image,
                           const cv::Mat &curr_image,
                           //    const Eigen::Matrix3d &rotation,
                           std::vector<unsigned long long int> &keypoints_id,
                           std::vector<cv::Point2f> &pre_keypoints,
                           std::vector<cv::Point2f> &curr_keypoints)
{
    static cv::Size patch_size(21, 21);
    // std::vector<cv::Mat> pre_img_pyramid, curr_img_pyramid;
    // cv::buildOpticalFlowPyramid(
    //     pre_image,
    //     pre_img_pyramid,
    //     patch_size,
    //     3,
    //     true,
    //     cv::BORDER_REFLECT_101,
    //     cv::BORDER_CONSTANT,
    //     false);

    // cv::buildOpticalFlowPyramid(
    //     curr_image,
    //     curr_img_pyramid,
    //     patch_size,
    //     3,
    //     true,
    //     cv::BORDER_REFLECT_101,
    //     cv::BORDER_CONSTANT,
    //     false);
    // cv::Mat mask(curr_image.rows, curr_image.cols, CV_8U, cv::Scalar(1));
    // std::vector<cv::Rect> objects;
    // CarDetector.detectMultiScale(curr_image, objects);
    // for (int i = 0; i < objects.size(); i++)
    // {
    //     mask(objects[i]) = 0;
    // }
    // objects.clear();
    // BodyDetector.detectMultiScale(curr_image, objects);
    // for (int i = 0; i < objects.size(); i++)
    // {
    //     mask(objects[i]) = 0;
    // }

    std::vector<uchar> status; // status of tracked features
    std::vector<float> err;    // error in tracking
    cv::calcOpticalFlowPyrLK(pre_image,
                             curr_image,
                             pre_keypoints,
                             curr_keypoints,
                             status,
                             err,
                             patch_size,
                             3);
    int k = 0;
    for (int i = 0; i < curr_keypoints.size(); i++)
    {
        if (status[i] &&
            curr_keypoints[i].x > 10.0 && curr_keypoints[i].y > 10.0 &&
            curr_keypoints[i].x < curr_image.cols - 10.0 && curr_keypoints[i].y < curr_image.rows - 10.0 /* &&
            mask.at<int>(int(curr_keypoints[i].x), int(curr_keypoints[i].y)) != 0*/
        )
        {
            keypoints_id[k] = keypoints_id[i];
            curr_keypoints[k] = curr_keypoints[i];
            pre_keypoints[k] = pre_keypoints[i];
            k++;
        }
    }
    keypoints_id.resize(k);
    curr_keypoints.resize(k);
    pre_keypoints.resize(k);
    if (curr_keypoints.size() == 0)
    {
        LOG(ERROR) << "TRACK LOST" << std::endl;
        return;
    }
    OutlierRemove(pre_keypoints, curr_keypoints, keypoints_id);
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
    std::vector<uchar> mask;
    matcher_->match(descriptors1, descriptors2, tmp_matches, mask);
    // double min_dist = min_element(tmp_matches.begin(), tmp_matches.end(),
    //                               [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; })
    //                       ->distance;
    double mean_dist = std::accumulate(tmp_matches.begin(), tmp_matches.end(), 0.0, [](double mean_dist, cv::DMatch match) {
        return mean_dist + match.distance;
    });
    float threshold_distance = mean_dist / tmp_matches.size();
    threshold_distance = default_min_distance > threshold_distance ? default_min_distance : threshold_distance;

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
    else if (matches.size() > 120)
    {
        threshold_distance *= 0.75;
        matches.clear();
        goto LEAP;
        LOG(WARNING) << "threshold_distance is " << threshold_distance
                     << " matched point count is " << matches.size() << std::endl;
    }

    // OutlierRemove(keypoints1, keypoints2, matches);
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
void ImageProcess::OutlierRemove(std::vector<cv::Point2f> &keypoints1,
                                 std::vector<cv::Point2f> &keypoints2,
                                 //  const Eigen::Matrix3d &rotation,
                                 std::vector<unsigned long long int> &keypoints_id)
{
    std::vector<uchar> ransac_status;
    cv::Mat Homography = cv::findFundamentalMat(keypoints1, keypoints2, ransac_status, cv::FM_RANSAC, 3.0);
    LOG_IF(ERROR, (ransac_status.size() != keypoints1.size())) << " ERROR ransac size != matches size" << std::endl;
    int k = 0;
    for (size_t i = 0; i < ransac_status.size(); i++)
    {
        if (ransac_status[i] != 0)
        {
            keypoints_id[k] = keypoints_id[i];
            keypoints1[k] = keypoints1[i];
            keypoints2[k] = keypoints2[i];
            k++;
        }
    }
    keypoints_id.resize(k);
    keypoints1.resize(k);
    keypoints2.resize(k);

    return;
}

/**
 * @brief 归一化关键点的坐标，计算得到尺度因子
 * @param pts1：上一时刻的关键点位置
 * @param pts2:当前时刻跟踪匹配到的关键点位置
 * @return scaling_factor：尺度因子
 *
 */
void ImageProcess::rescalePoints(std::vector<cv::Point2f> &pts1,
                                 std::vector<cv::Point2f> &pts2,
                                 float &scaling_factor)
{

    scaling_factor = 0.0f;

    // 将所有关键点的模长相加
    for (int i = 0; i < pts1.size(); ++i)
    {
        scaling_factor += sqrt(pts1[i].dot(pts1[i]));
        scaling_factor += sqrt(pts2[i].dot(pts2[i]));
    }

    // 为了采用乘法，这里其实采用的计算方式是倒数
    scaling_factor = (pts1.size() + pts2.size()) /
                     scaling_factor * sqrt(2.0f);

    // 关键点的归一化处理
    // pts1 = pts1/（sum(sqrt(pts.dot(pts)))/(pts.size*sqrt(2)）
    for (int i = 0; i < pts1.size(); ++i)
    {
        pts1[i] *= scaling_factor;
        pts2[i] *= scaling_factor;
    }

    return;
}

void ImageProcess::TwoPointRansac(
    const std::vector<cv::Point2f> &pts1,
    const std::vector<cv::Point2f> &pts2,
    const cv::Matx33f &R_p_c,
    const cv::Mat &intrinsics,
    const cv::Mat &distortion_coeffs,
    const double &inlier_error,
    const double &success_probability,
    std::vector<int> &inlier_markers)
{

    // Check the size of input point size.
    if (pts1.size() != pts2.size())
        LOG(FATAL) << "Sets of different size" << pts1.size() << "\t" << pts2.size() << std::endl;

    // 平均焦距 f_a = (fx+fy)/2
    // norm_pixel_unit = 1 / f_a 表示一个像素点的归一化坐标值偏差
    double norm_pixel_unit = 2.0 / (intrinsics.at<double>(0, 0) + intrinsics.at<double>(1, 1));
    int iter_num = static_cast<int>(ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));

    // Initially, mark all points as inliers.
    // 对所有的关键点赋予一个判断是否为内点的标志位
    // 初始化的inlier_markers都置为1
    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 1);

    // Undistort all the points.
    // 对前后时刻所有的关键点进行去畸变操作
    std::vector<cv::Point2f> pts1_undistorted(pts1.size()), pts2_undistorted(pts2.size());
    cv::undistortPoints(pts1, pts1_undistorted, intrinsics, distortion_coeffs);
    cv::undistortPoints(pts2, pts2_undistorted, intrinsics, distortion_coeffs);

    // Compenstate the points in the previous image with
    // the relative rotation.
    // 乘上帧间的旋转使上一时刻与当前时刻的关键点之间只有平移量
    for (auto &pt : pts1_undistorted)
    {
        cv::Vec3f pt_h(pt.x, pt.y, 1.0f);
        cv::Vec3f pt_hc = R_p_c * pt_h;
        pt.x = pt_hc[0];
        pt.y = pt_hc[1];
    }

    // Normalize the points to gain numerical stability.
    // 归一化关键点（去除模长）从而来提高数值稳定性
    float scaling_factor = 0.0f;
    rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
    norm_pixel_unit *= scaling_factor;

    // Compute the difference between previous and current points,
    // which will be used frequently later.
    // 计算前后两帧匹配的关键点的差值
    std::vector<cv::Point2d> pts_diff(pts1_undistorted.size());
    for (int i = 0; i < pts1_undistorted.size(); ++i)
        pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

    // Mark the point pairs with large difference directly.
    // BTW, the mean distance of the rest of the point pairs
    // are computed.
    // 计算关键点差值的中间值，将差值大于阈值的点对视为外点剔除
    double mean_pt_distance = 0.0;
    int raw_inlier_cntr = 0;
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
        // 25 pixel distance is a pretty large tolerance for normal motion.
        // However, to be used with aggressive motion, this tolerance should
        // be increased significantly to match the usage.
        // 阈值设为50个像素差
        if (distance > 5.0 * norm_pixel_unit)
        {
            inlier_markers[i] = 0;
        }
        else
        {
            mean_pt_distance += distance;
            ++raw_inlier_cntr;
        }
    }
    mean_pt_distance /= raw_inlier_cntr;

    // If the current number of inliers is less than 3, just mark
    // all input as outliers. This case can happen with fast
    // rotation where very few features are tracked.
    if (raw_inlier_cntr < 3)
    {
        for (auto &marker : inlier_markers)
            marker = 0;
        return;
    }

    // Before doing 2-point RANSAC, we have to check if the motion
    // is degenerated, meaning that there is no translation between
    // the frames, in which case, the model of the RANSAC does not
    // work. If so, the distance between the matched points will
    // be almost 0.
    // 检查运动是否退化，退化则表示帧间的平移量几乎为0
    // 平移量为0则匹配点对之间的距离几乎为0
    // 平均的差值小于1个像素则认为退化，只需简单比较设置的阈值来判断外点
    //if (mean_pt_distance < inlier_error*norm_pixel_unit) {
    if (mean_pt_distance < norm_pixel_unit)
    {
        LOG(ERROR) << 1.0 << "Degenerated motion..." << std::endl;
        for (int i = 0; i < pts_diff.size(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            // 关键点之间的距离大于阈值时将其视为外点
            if (sqrt(pts_diff[i].dot(pts_diff[i])) >
                inlier_error * norm_pixel_unit)
                inlier_markers[i] = 0;
        }
        return;
    }

    // In the case of general motion, the RANSAC model can be applied.
    // The three column corresponds to tx, ty, and tz respectively.
    //
    Eigen::MatrixXd coeff_t(pts_diff.size(), 3);
    for (int i = 0; i < pts_diff.size(); ++i)
    {
        coeff_t(i, 0) = pts_diff[i].y;
        coeff_t(i, 1) = -pts_diff[i].x;
        coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y -
                        pts1_undistorted[i].y * pts2_undistorted[i].x;
    }

    // 找到被认为是内点的匹配关键点对的索引
    std::vector<int> raw_inlier_idx;
    for (int i = 0; i < inlier_markers.size(); ++i)
    {
        if (inlier_markers[i] != 0)
            // 将内点位置索引保存
            raw_inlier_idx.push_back(i);
    }

    //
    std::vector<int> best_inlier_set;
    double best_error = 1e10;
    // ros包，随机数的生成
    std::srand(time(0));

    // 执行两点RANSAC
    for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx)
    {
        // Randomly select two point pairs.
        // Although this is a weird way of selecting two pairs, but it
        // is able to efficiently avoid selecting repetitive pairs.
        // 随机选择两个点对pair_idx1和pair_idx2（索引）
        // 先从保存内点索引的raw_inlier_idx中随机产生一个位置索引
        // 随机产生一个索引差值
        int random_index = std::rand() % raw_inlier_idx.size();

        int pair_idx1 = raw_inlier_idx[random_index];
        int idx_diff = std::rand() % raw_inlier_idx.size() + 1;
        // 产生第二个索引位置
        // 如果索引pair_idx1加上索引差超过最大值，则减去索引最大值
        int pair_idx2 = pair_idx1 + idx_diff < raw_inlier_idx.size() ? pair_idx1 + idx_diff : pair_idx1 + idx_diff - raw_inlier_idx.size();

        // Construct the model;
        //
        Eigen::Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
        Eigen::Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
        Eigen::Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
        std::vector<double> coeff_l1_norm(3);
        coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
        coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
        coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
        int base_indicator = min_element(coeff_l1_norm.begin(),
                                         coeff_l1_norm.end()) -
                             coeff_l1_norm.begin();

        Eigen::Vector3d model(0.0, 0.0, 0.0);
        if (base_indicator == 0)
        {
            Eigen::Matrix2d A;
            A << coeff_ty, coeff_tz;
            Eigen::Vector2d solution = A.inverse() * (-coeff_tx);
            model(0) = 1.0;
            model(1) = solution(0);
            model(2) = solution(1);
        }
        else if (base_indicator == 1)
        {
            Eigen::Matrix2d A;
            A << coeff_tx, coeff_tz;
            Eigen::Vector2d solution = A.inverse() * (-coeff_ty);
            model(0) = solution(0);
            model(1) = 1.0;
            model(2) = solution(1);
        }
        else
        {
            Eigen::Matrix2d A;
            A << coeff_tx, coeff_ty;
            Eigen::Vector2d solution = A.inverse() * (-coeff_tz);
            model(0) = solution(0);
            model(1) = solution(1);
            model(2) = 1.0;
        }

        // Find all the inliers among point pairs.
        Eigen::VectorXd error = coeff_t * model;

        std::vector<int> inlier_set;
        for (int i = 0; i < error.rows(); ++i)
        {
            if (inlier_markers[i] == 0)
                continue;
            if (std::abs(error(i)) < inlier_error * norm_pixel_unit)
                inlier_set.push_back(i);
        }

        // If the number of inliers is small, the current
        // model is probably wrong.
        if (inlier_set.size() < 0.2 * pts1_undistorted.size())
            continue;

        // Refit the model using all of the possible inliers.
        Eigen::VectorXd coeff_tx_better(inlier_set.size());
        Eigen::VectorXd coeff_ty_better(inlier_set.size());
        Eigen::VectorXd coeff_tz_better(inlier_set.size());
        for (int i = 0; i < inlier_set.size(); ++i)
        {
            coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
            coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
            coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
        }

        Eigen::Vector3d model_better(0.0, 0.0, 0.0);
        if (base_indicator == 0)
        {
            Eigen::MatrixXd A(inlier_set.size(), 2);
            A << coeff_ty_better, coeff_tz_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
            model_better(0) = 1.0;
            model_better(1) = solution(0);
            model_better(2) = solution(1);
        }
        else if (base_indicator == 1)
        {
            Eigen::MatrixXd A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_tz_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
            model_better(0) = solution(0);
            model_better(1) = 1.0;
            model_better(2) = solution(1);
        }
        else
        {
            Eigen::MatrixXd A(inlier_set.size(), 2);
            A << coeff_tx_better, coeff_ty_better;
            Eigen::Vector2d solution =
                (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
            model_better(0) = solution(0);
            model_better(1) = solution(1);
            model_better(2) = 1.0;
        }

        // Compute the error and upate the best model if possible.
        Eigen::VectorXd new_error = coeff_t * model_better;

        double this_error = 0.0;
        for (const auto &inlier_idx : inlier_set)
            this_error += std::abs(new_error(inlier_idx));
        this_error /= inlier_set.size();

        if (inlier_set.size() > best_inlier_set.size())
        {
            best_error = this_error;
            best_inlier_set = inlier_set;
        }
    }

    // Fill in the markers.
    inlier_markers.clear();
    inlier_markers.resize(pts1.size(), 0);
    for (const auto &inlier_idx : best_inlier_set)
        inlier_markers[inlier_idx] = 1;

    //printf("inlier ratio: %lu/%lu\n",
    //    best_inlier_set.size(), inlier_markers.size());

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
    cv::Mat Homography = cv::findFundamentalMat(keypointf1, keypointf2, ransac_status, cv::FM_RANSAC, 25.0);
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
