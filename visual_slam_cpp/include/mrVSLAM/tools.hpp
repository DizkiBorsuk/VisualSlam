/**
 * @file tools.hpp
 * @author mrostocki
 * @brief tools used in various slam modules, like triangulation, plots etc.
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "mrVSLAM/common_includes.hpp"
#include <matplot/matplot.h>
#include "mrVSLAM/frame.hpp"

namespace mrVSLAM
{
    /**
     * @brief Function that calculates 3d position of the point in the world from stereo camera using SVD
     *
     * @param l_cam_Rt - left camera Rt matrix
     * @param r_cam_Rt - right camera Rt matrix
     * @param points - set of coresponding points from stereo imgs, they must be given in camera coordinate system
     * @param out_point_pos - function output: 3d position of a point
     * @return true - return true if traingulation is succesful
     * @return false
     */
    inline bool triangulate(const Sophus::SE3d &l_cam_Rt, const Sophus::SE3d &r_cam_Rt,
                            const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &out_point_pos)
    {
        Eigen::MatrixXd A(4, 4); // A has to be Xmatrix to do svd
        Eigen::Matrix<double, 3, 4> Rt1 = l_cam_Rt.matrix3x4();
        Eigen::Matrix<double, 3, 4> Rt2 = r_cam_Rt.matrix3x4();

        A.block<1,4>(0,0) = points[0][0] * Rt1.row(2) - Rt1.row(0);
        A.block<1,4>(1,0) = points[0][1] * Rt1.row(2) - Rt1.row(1);

        A.block<1,4>(2,0) = points[1][0] * Rt2.row(2) - Rt2.row(0);
        A.block<1,4>(3,0) = points[1][1] * Rt2.row(2) - Rt2.row(1);


        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        out_point_pos = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if ((svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) && out_point_pos[2] > 0)
        {
            return true;
        }

        return false;
    }

    /**
     * @brief convert cv::Point to eigen vector
     *
     * @param point cv::Point2f / cv::Point3f
     * @return Eigen::Vector2d  / Eigen::Vector3d
     */
    inline Eigen::Vector2d convertToVec(const cv::Point2f point)
    {
        return Eigen::Vector2d(point.x, point.y);
    }

    inline Eigen::Vector3d convertToVec(const cv::Point3f point)
    {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    inline void calculate_error(const std::vector<Eigen::Matrix<double, 3,4>> &poses,
                                const std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses,
                                const float resize_opt, const int seq, ResultStruct &out_struct)
    {
        float seq0_dist = 3724.18f;
        float seq6_dist = 1232.87f;
        float seq7_dist = 694.69f;

        if(poses.size()!=gt_poses.size())
        {
            fmt::print(fg(fmt::color::red), "size of poses vector doesn't match size of ground truth poses \n");
            fmt::print("poses.size() = {}, gt_poses.size() = {} \n", poses.size(), gt_poses.size());
            return;
        }

        float gt_x, gt_y, gt_z, x, y, z;
        std::vector<float> er_x, er_y, er_z;

        for(size_t i = 0; i < poses.size(); i++)
        {
            gt_x = gt_poses.at(i).coeff(0,3);
            gt_y = gt_poses.at(i).coeff(2,3);
            gt_z = gt_poses.at(i).coeff(1,3);

            x = poses.at(i).coeff(0,3)*resize_opt;
            y = poses.at(i).coeff(2,3)*resize_opt;
            z = poses.at(i).coeff(1,3)*resize_opt;

            er_x.emplace_back((gt_x - x)*(gt_x - x));
            er_y.emplace_back((gt_y - y)*(gt_y - y));
            er_z.emplace_back((gt_z - z)*(gt_z - z));
        }

        auto compare_abs = [](auto value1, auto value2) {return std::abs(value1) < std::abs(value2); };

        float sum_error_x = std::accumulate(er_x.begin(), er_x.end(), 0.0f);
        float sum_error_y = std::accumulate(er_y.begin(), er_y.end(), 0.0f);
        float sum_error_z = std::accumulate(er_z.begin(), er_z.end(), 0.0f);

        float mean_error_x = sqrt(sum_error_x/static_cast<float>(er_x.size()));
        float mean_error_y = sqrt(sum_error_y/static_cast<float>(er_y.size()));
        float mean_error_z = sqrt(sum_error_z/static_cast<float>(er_z.size()));
        float mean_error = (mean_error_x + mean_error_y + mean_error_z)/3.0f;

        float min_error_x = sqrt(*std::min_element(er_x.begin(), er_x.end(), compare_abs));
        float min_error_y = sqrt(*std::min_element(er_y.begin(), er_y.end(), compare_abs));
        float min_error_z = sqrt(*std::min_element(er_z.begin(), er_z.end(), compare_abs));

        float max_error_x = sqrt(*std::max_element(er_x.begin(), er_x.end(), compare_abs));
        float max_error_y = sqrt(*std::max_element(er_y.begin(), er_y.end(), compare_abs));
        float max_error_z = sqrt(*std::max_element(er_z.begin(), er_z.end(), compare_abs));

        float percent_error = 0.0f;

        switch (seq)
        {
        case 0:
            percent_error = (mean_error/seq0_dist) * 100.0f;
            fmt::print("error calculation: using seq 0 \n");
            break;
        case 6:
            percent_error = (mean_error/seq6_dist) * 100.0f;
            fmt::print("error calculation: using seq 6 \n");
            break;
        case 7:
            percent_error = (mean_error/seq7_dist) * 100.0f;
            fmt::print("error calculation: using seq 7 \n");
            break;
        default:
            break;
        }

        out_struct.mean_error = mean_error;
        out_struct.percent_error = percent_error;
        out_struct.mean_error_x = mean_error_x;
        out_struct.mean_error_y = mean_error_y;
        out_struct.mean_error_z = mean_error_z;
        out_struct.max_error_x = max_error_x;
        out_struct.max_error_y = max_error_y;
        out_struct.max_error_z = max_error_z;
        out_struct.min_error_x = min_error_x;
        out_struct.min_error_y = min_error_y;
        out_struct.min_error_z = min_error_z;

        fmt::print("mean error = {} \nmean_error_x = {}, mean_error_y = {}, mean_error_z = {} \n", mean_error, mean_error_x, mean_error_y, mean_error_z);
        fmt::print("percent error = {} \n", percent_error);
    }

    inline void calculate_kf_error(const std::vector<Eigen::Matrix<double, 3,4>> &kf_poses,
                                   const std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &kf_gt_poses,
                                   const float resize_opt, const int seq, ResultStruct &out_struct)
    {
        float seq0_dist = 3724.18f;
        float seq6_dist = 1232.87f;
        float seq7_dist = 694.69f;

        if(kf_poses.size()!=kf_gt_poses.size())
        {
            fmt::print(fg(fmt::color::red), "size of poses vector doesn't match size of ground truth poses \n");
            fmt::print("poses.size() = {}, gt_poses.size() = {} \n", kf_poses.size(), kf_gt_poses.size());
            return;
        }

        float gt_x, gt_y, gt_z, x, y, z;
        std::vector<float> er_x, er_y, er_z;

        for(size_t i = 0; i < kf_poses.size(); i++)
        {
            gt_x = kf_gt_poses.at(i).coeff(0,3);
            gt_y = kf_gt_poses.at(i).coeff(2,3);
            gt_z = kf_gt_poses.at(i).coeff(1,3);

            x = kf_poses.at(i).coeff(0,3)*resize_opt;
            y = kf_poses.at(i).coeff(2,3)*resize_opt;
            z = kf_poses.at(i).coeff(1,3)*resize_opt;

            er_x.emplace_back((gt_x - x)*(gt_x - x));
            er_y.emplace_back((gt_y - y)*(gt_y - y));
            er_z.emplace_back((gt_z - z)*(gt_z - z));
        }

        float sum_error_x = std::accumulate(er_x.begin(), er_x.end(), 0.0f);
        float sum_error_y = std::accumulate(er_y.begin(), er_y.end(), 0.0f);
        float sum_error_z = std::accumulate(er_z.begin(), er_z.end(), 0.0f);

        float mean_error_x = sqrt(sum_error_x/static_cast<float>(er_x.size()));
        float mean_error_y = sqrt(sum_error_y/static_cast<float>(er_y.size()));
        float mean_error_z = sqrt(sum_error_z/static_cast<float>(er_z.size()));
        float mean_error = (mean_error_x + mean_error_y + mean_error_z)/3.0f;

        float percent_error = 0.0f;
        switch (seq)
        {
        case 0:
            percent_error = (mean_error/seq0_dist) * 100.0f;
            fmt::print("error calculation: using seq 0 \n");
            break;
        case 6:
            percent_error = (mean_error/seq6_dist) * 100.0f;
            fmt::print("error calculation: using seq 6 \n");
            break;
        case 7:
            percent_error = (mean_error/seq7_dist) * 100.0f;
            fmt::print("error calculation: using seq 7 \n");
            break;
        default:
            break;
        }

        out_struct.mean_kf_error = mean_error;
        out_struct.percent_kf_error = percent_error;
        out_struct.mean_kf_error_x = mean_error_x;
        out_struct.mean_kf_error_y = mean_error_y;
        out_struct.mean_kf_error_z = mean_error_z;
    }


    inline void calculate_time(const std::vector<float>& loop_times, ResultStruct &out_struct)
    {
        float sum = std::accumulate(loop_times.begin(), loop_times.end(), 0.0f);
        double mean = sum / static_cast<double>(loop_times.size());
        out_struct.mean_time = mean;
        out_struct.min_time = *std::min_element(loop_times.begin(), loop_times.end());
        out_struct.max_time = *std::max_element(loop_times.begin(), loop_times.end());

        fmt::print("mean time = {}, min time = {}, max_time = {} \n" , mean, out_struct.min_time, out_struct.max_time);
    }

    /**
     * @brief function that plots estimated and ground truth camera trajectory
     *
     * @param poses - camera poses [matrix 3x4] estimated by algorithm
     * @param gt_poses - true camera poses
     * @param resize_opt - resize img option that
     */
    inline void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses,
                          std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resize_opt,
                          std::string title)
    {
        std::vector<double> gt_x, gt_y, x, y;

        for(auto& pose : poses)
        {
            x.emplace_back(pose.coeff(0,3)*resize_opt);
            y.emplace_back(pose.coeff(2,3)*resize_opt);
        }

        for(auto gt_pose : gt_poses)
        {
           gt_x.emplace_back(gt_pose.coeff(0,3));
           gt_y.emplace_back(gt_pose.coeff(2,3));
        }

        // set figure size anf color
        auto fig = matplot::figure();
        fig->width(fig->width()*1.3);
        fig->height(fig->height()*1.3);
        fig->color("white");

        matplot::plot(gt_x, gt_y)->line_width(2);
        matplot::hold(matplot::on);
        matplot::plot(x, y, "r--")->line_width(2);
        matplot::xlabel("x [m]");
        matplot::ylabel("y [m]");
        matplot::legend({"prawdziwa trasa", "estymowana trasa"});
        matplot::title(title);
        matplot::show();
    }

    inline void plotTrajectory(std::vector<Eigen::Matrix<double, 3,4>> &poses,  float resize_opt)
    {
        std::vector<double> x, y;

        for(std::size_t i = 0; i < poses.size(); i++)
        {
            x.emplace_back(poses.at(i).coeff(0,3)*resize_opt);
            y.emplace_back(poses.at(i).coeff(2,3)*resize_opt);
        }

        // set figure size anf color
        auto fig = matplot::figure();
        fig->width(fig->width()*1.3);
        fig->height(fig->height()*1.3);
        fig->color("white");

        matplot::plot(x, y, "b")->line_width(2);
        matplot::xlabel("x [m]");
        matplot::ylabel("y [m]");
        matplot::show();
    }

    inline void plotLoopClosingMatches(std::vector<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>> matched_keyframes,
                                       std::vector<Eigen::Matrix<double, 3,4>> &poses,  float resize_opt)
    {
        std::vector<double> x, y;

        for(std::size_t i = 0; i < poses.size(); i++)
        {
            x.emplace_back(poses.at(i).coeff(0,3)*resize_opt);
            y.emplace_back(poses.at(i).coeff(2,3)*resize_opt);
        }

        // set figure size anf color
        auto fig = matplot::figure();
        fig->width(fig->width()*1.3);
        fig->height(fig->height()*1.3);
        fig->color("white");

        matplot::plot(x, y, "k")->line_width(2);
        matplot::xlabel("x [m]");
        matplot::ylabel("y [m]");
        matplot::hold(matplot::on);

        for (auto& kf_pair : matched_keyframes)
        {
            auto older_kf_pose = kf_pair.first->getPose().inverse();
            auto newer_kf_pose = kf_pair.second->getPose().inverse();
            std::string kf_info = std::to_string(kf_pair.first->kf_id) + " + " + std::to_string(kf_pair.second->kf_id);

            std::vector<double> x , y;
            x.emplace_back(older_kf_pose.matrix3x4().coeff(0,3)*resize_opt);
            x.emplace_back(newer_kf_pose.matrix3x4().coeff(0,3)*resize_opt);
            y.emplace_back(older_kf_pose.matrix3x4().coeff(2,3)*resize_opt);
            y.emplace_back(newer_kf_pose.matrix3x4().coeff(2,3)*resize_opt);

            matplot::plot(x, y, "--o")->line_width(1.5);
            matplot::text(x[0], y[0], kf_info);
        }

        matplot::show();
    }

    /**
     * @brief function to plot loop times and calculate mean/min times
     *
     * @param loopTimes - vector of times of each loop
     * @param results_object
     */
    inline void plotPerformance(const std::vector<int> &loopTimes, ResultStruct &results_object)
    {
        int sum = std::accumulate(loopTimes.begin(), loopTimes.end(), 0.0);
        double mean = sum/static_cast<double>(loopTimes.size());

        results_object.mean_time = mean;
        results_object.min_time = *std::min_element(loopTimes.begin(), loopTimes.end());

        std::cout << "mean time = " << mean << "ms \n";
        std::cout << "min value = " << *std::min_element(loopTimes.begin(), loopTimes.end()) << "\n";

        auto fig = matplot::figure();
        fig->width(fig->width()*1.3);
        fig->height(fig->height()*1.3);
        fig->color("white");
        matplot::plot(loopTimes);
        matplot::xlabel(" iteracja []");
        matplot::ylabel("czas [ms]");
        matplot::show();
    }

    template <typename E>
    constexpr auto to_underlying(E e) noexcept
    {
        return static_cast<std::underlying_type_t<E>>(e);
    }

} //! end of namespace
