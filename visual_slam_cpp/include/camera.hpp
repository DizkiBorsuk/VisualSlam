#pragma once 
#include "common_includes.hpp"

namespace mrVSLAM
{
    class Camera
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // camera calibration parameters  
        double fx = 0, fy = 0, cx = 0, cy = 0, baseline = 0; 
        cv::Point2d principialPoint = {0,0}; 
        // camera intrinsic and extrinsic 
        Eigen::Matrix3d K_eigen, R_eigen;  
        cv::Matx<double,3, 3> K, R; // K - intrinsic matrix, R - camera rotation matrix 
        cv::Matx<double, 3, 1> t;// t - camera translation vector 

        void setCamera(const Eigen::Matrix<double,3,4> &projectionMatrix) noexcept; 

        // Member functions 

        std::array<double,3> world2camera(); 
        std::array<double,3> camera2world();

        std::array<double,2> camera2pixel(); 
        std::array<double,3> pixel2camera(); 
        std::array<double,2> world2pixel(); 

    }; 

    double getStereoBaseline(const cv::Matx<double, 3, 1> &t1, const cv::Matx<double, 3, 1> &t2) noexcept; 

}