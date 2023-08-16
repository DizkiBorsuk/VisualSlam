#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    void getTransformationMatrix(const cv::Matx33d &R,const cv::Matx31d &t, cv::Matx44d &outT); 

    
    void getTransformationMatrix(const Eigen::Matrix3d &R, const Eigen::Vector4d &t, Eigen::Matrix<double,3,4> &outT); 


    // Eigen::Matrix3d findFundamentalMatrix(const std::vector<std::vector<cv::Point2f>> &matched_points); 
    // Eigen::Matrix3d findEssentialMatrix(const Eigen::Matrix3d &intrinsicMatrix, const Eigen::Matrix3d &fundamentalMatrix); 
    //  void decomposeProjectionMatrix(const Eigen::Matrix<double,3,4> &projectionMatrix, 
    //                                const Eigen::Matrix3d &K, 
    //                                const Eigen::Matrix3d &R, 
    //                                const Eigen::Vector3d &t); 



}