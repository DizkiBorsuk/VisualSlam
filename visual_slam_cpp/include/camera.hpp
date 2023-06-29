#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class Camera
    {
        static double fx, fy, cx, cy, baseline; 
        cv::Point2f principialPoint; 
        static Eigen::Matrix3d K_eigen;  
        static cv::Mat K_mat; 

        Camera();
        Camera(const Eigen::Matrix<double,3,4> &projectionMatrix, const Eigen::Matrix<double,3,4> &projectionMatrixRight); 
    }; 

    Camera::Camera(const Eigen::Matrix<double,3,4> &projectionMatrix, const Eigen::Matrix<double,3,4> &projectionMatrixRight)
    {
        cv::Mat P; 
        cv::eigen2cv(projectionMatrix, P); 
        cv::decomposeProjectionMatrix(P, K, R, t_h); 
        t = (cv::Mat_<double>(3,1) <<0,0,0); 
        cx = projectionMatrix.coeff(0,2); 
        cy = projectionMatrix.coeff(1,2); 
        fx = projectionMatrix.coeff(0,0); 
        fy = projectionMatrix.coeff(1,1); 
        std::cout << "cx = " << cx <<"\n cy = " << cy <<"\n f = " <<fx <<"\n"; 
        std::cout << "t = " << t << "R = " << R << "\n K = " << K << "\n"
    }
}


