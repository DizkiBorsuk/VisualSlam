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
        Eigen::Matrix3d K_eigen;  
        Eigen::Matrix4d extrinsics; //homogenous extrinsic matrix

        cv::Matx<double,3, 3> K, R; // K - intrinsic matrix, R - camera rotation matrix 
        cv::Matx<double, 3, 1> t;// t - camera translation vector 

        void setCamera(const Eigen::Matrix<double,3,4> &projectionMatrix) noexcept; 

        // Member functions 

        Eigen::Vector3d world2camera(const Eigen::Vector3d &point_in_world, const Eigen::Matrix4d Tcw); // std::array<double,3> world2camera(); 
        Eigen::Vector3d camera2world(const Eigen::Vector3d &point_in_camera, const Eigen::Matrix4d Twc); // std::array<double,3> camera2world();

        Eigen::Vector2d camera2pixel(const Eigen::Vector3d &point_in_camera);// std::array<double,2> camera2pixel(); 
        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &pixel_positon, double alfa); // std::array<double,3> pixel2camera(); 
        Eigen::Vector2d world2pixel(const Eigen::Vector3d &point_in_world, const Eigen::Matrix4d &Tcw) // std::array<double,2> world2pixel(); 
        {
            return camera2pixel(world2camera(point_in_world, Tcw)); 
        }

    }; 

    double getStereoBaseline(const cv::Matx<double, 3, 1> &t1, const cv::Matx<double, 3, 1> &t2) noexcept; 

}