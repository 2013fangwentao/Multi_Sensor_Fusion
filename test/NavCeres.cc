/*
** NavCeres.cc for mscnav in /media/fwt/Data/program/mscnav/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Aug 12 ä¸å11:13:57 2019 little fang
** Last update Thu Aug 21 下午7:54:45 2019 little fang
*/

#include "ceres/ceres.h"
#include "opencv2/core/core.hpp"
#include "navlog.hpp"
#include "navattitude.hpp"
#include "navbase.hpp"
#include "camera/imageprocess.h"
#include <fstream>

using namespace mscnav::camera;
// // using namespace cv;
//
// // struct cost_function_define
// // {
// //     cost_function_define(Point3d p1, Point3d p2) : _p1(p1), _p2(p2) {}
// //     template <typename T>
// //     bool operator()(const T *const cere_r, const T *const cere_t, T *residual) const
// //     {
// //         T p_1[3];
// //         T p_2[3];
// //         p_1[0] = T(_p1.x);
// //         p_1[1] = T(_p1.y);
// //         p_1[2] = T(_p1.z);
// //         ceres::AngleAxisRotatePoint(cere_r, p_1, p_2);
// //         p_2[0] = p_2[0] + cere_t[0];
// //         p_2[1] = p_2[1] + cere_t[1];
// //         p_2[2] = p_2[2] + cere_t[2];
// //         const T x = p_2[0] / p_2[2];
// //         const T y = p_2[1] / p_2[2];
// //         const T u = x * 520.9 + 325.1;
// //         const T v = y * 521.0 + 249.7;
// //         T p_3[3];
// //         p_3[0] = T(_p2.x);
// //         p_3[1] = T(_p2.y);
// //         p_3[2] = T(_p2.z);
// //         const T x1 = p_3[0] / p_3[2];
// //         const T y1 = p_3[1] / p_3[2];
// //         const T u1 = x1 * 520.9 + 325.1;
// //         const T v1 = y1 * 521.0 + 249.7;
// //         residual[0] = u - u1;
// //         residual[1] = v - v1;
// //         return true;
// //     }
// //     Point3d _p1, _p2;
// // };
// // int main(int argc, char const *argv[])
// // {
// //     std::vector<cv::Point3d> image1_3d, image2_3d;
// //     double cere_r[3], cere_t[3];
// //     ceres::Problem problem;
// //     for (int i = 0; i < image1_3d.size(); i++)
// //     {
// //         ceres::CostFunction *costfunction =
// //         new ceres::AutoDiffCostFunction<cost_function_define, 2, 3, 3>(new cost_function_define(image2_3d[i], image1_3d[i]));
// //         problem.AddResidualBlock(costfunction, NULL, cere_r, cere_t);
// //     }
// //     ceres::Solver::Options option;
// //     option.linear_solver_type = ceres::DENSE_SCHUR;
// //     //è¾åºè¿­ä»£ä¿¡æ¯å°å±å¹
// //     option.minimizer_progress_to_stdout = true;
// //     //æ¾ç¤ºä¼åä¿¡æ¯
// //     ceres::Solver::Summary summary;
// //     //å¼å§æ±è§£
// //     ceres::Solve(option, &problem, &summary);
// //     //æ¾ç¤ºä¼åä¿¡æ¯
// //     std::cout << summary.BriefReport() << std::endl;
// //     return 0;
// // }

struct ReprojectionError
{
    ReprojectionError(const cv::Point2f &measure_point_uv,
                             const Eigen::Isometry3d &cam0_cami_transform) : image_uv(measure_point_uv),
                                                                             cam0_cami(cam0_cami_transform) {}
    template <typename T>
    bool operator()(const T *const feature_point_world, T *residuals)
    {
        Eigen::Matrix<T, 3, 1> point;
        point << feature_point_world[0], feature_point_world[1], feature_point_worldp[2];

        point = cam0_cami * point;
        double point_reproject_image[2] = {point(0) / point(2), point(1) / point(2)};
        residuals[0] = point_reproject_image[0] - image_uv[0];
        residuals[0] = point_reproject_image[1] - image_uv[2];
    }

    static ceres::CostFunction *Create(const cv::Point2f &measure_point_uv,
                                       const Eigen::Isometry3d &cam0_cami_transform)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
            new ReprojectionError(measure_point_uv, cam0_cami_transform)));
    }

private:
    cv::Point2f image_uv;
    Eigen::Isometry3d cam0_cami;
};

int main(int argc, char const *argv[])
{
    if (argc != 3)
    {
        std::cout << "input error" << std::endl
                  << "test_nav_ceres configure_file image_path" << std::endl;
        return 0;
    }
    utiltool::LogInit(argv[0], "./log/");
    std::ifstream ifs_configure_file(argv[1]);
    std::string image_path = argv[2];
    if (!ifs_configure_file)
    {
        LOG(ERROR) << "load configure file failed" << std::endl;
        return 0;
    }
    std::map<std::string, Eigen::Isometry3d> image_map;
    while (true)
    {
        std::string line;
        std::getline(ifs_configure_file, line);
        auto dat = utiltool::TextSplit(line, "\\s+");
    }

    ceres::Problem problem;

    return 0;
}
