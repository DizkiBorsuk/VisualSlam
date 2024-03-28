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

    /**
     * @brief function that plots estimated and ground truth camera trajectory 
     * 
     * @param poses - camera poses [matrix 3x4] estimated by algorithm
     * @param gt_poses - true camera poses 
     * @param resize_opt - resize img option that 
     */
    inline void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resize_opt)
    {
        std::vector<double> gt_x, gt_y, x, y; 
 
        for(std::size_t i = 0; i < poses.size(); i++)
        {
            gt_x.emplace_back(gt_poses.at(i).coeff(0,3));  
            gt_y.emplace_back(gt_poses.at(i).coeff(2,3));

            x.emplace_back(poses.at(i).coeff(0,3)*resize_opt);  
            y.emplace_back(poses.at(i).coeff(2,3)*resize_opt);
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
        double mean = sum/loopTimes.size(); 

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

 