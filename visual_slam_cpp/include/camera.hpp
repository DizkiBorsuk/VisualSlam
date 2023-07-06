#pragma once 
#include "system.hpp"

namespace mrVSLAM
{

    class Camera
    {
    public: 

        static std::shared_ptr<Camera> ptr; 

        // camera calibration parameters 
        double fx = 0, fy = 0, cx = 0, cy = 0; 
        cv::Point2f principialPoint; 
        // camera intrinsic and extrinsic 
        Eigen::Matrix3d K_eigen, R_eigen;  
        cv::Mat K_mat, R_mat, t_mat; 

        Camera(const Eigen::Matrix<double,3,4> &projectionMatrix); 


    }; 

}


