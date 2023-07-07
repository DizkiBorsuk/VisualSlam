#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class Camera
    {
    public: 
        // camera calibration parameters 
        double fx = 0, fy = 0, cx = 0, cy = 0; 
        cv::Point2d principialPoint = {0,0}; 
        // camera intrinsic and extrinsic 
        Eigen::Matrix3d K_eigen, R_eigen;  
        cv::Mat K_mat, R_mat, t_mat; 

        void setCamera(const Eigen::Matrix<double,3,4> &projectionMatrix); 
    }; 

    double getStereoBaseline(const cv::Mat &t1, const cv::Mat &t2); 

}


