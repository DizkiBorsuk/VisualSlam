#pragma once
#include "myslam/common_include.hpp"
#include "opencv4/opencv2/dnn.hpp"

namespace myslam {

    inline bool triangulation(const Sophus::SE3d cam_l_Rt,const Sophus::SE3d cam_r_Rt , const std::vector<Eigen::Vector3d> points, Eigen::Vector3d &out_point_pos) {
    
        Eigen::MatrixXd A(4, 4);
        Eigen::Matrix<double, 3, 4> Rt1 = cam_l_Rt.matrix3x4();
        Eigen::Matrix<double, 3, 4> Rt2 = cam_r_Rt.matrix3x4();

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


    inline Eigen::Vector2d toVec2(const cv::Point2f p) { return Eigen::Vector2d(p.x, p.y); }

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resize_opt);
    void plotPoses(std::vector<Eigen::Matrix<double, 3,4>> &poses);

    void plotPerformance(std::vector<int> loopTimes);
    void calculate_error(std::vector<Eigen::Matrix<double, 3,4>> &poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> &gt_poses, float resieze_opt); 

    inline std::vector<std::string> getNNModelOutput(const cv::dnn::Net &net); 
}



