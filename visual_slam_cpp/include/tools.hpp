#pragma once 
#include "system.hpp"

namespace mrVSLAM::utils 
{
    void getTransformationMatrix(cv::Matx33d R, cv::Matx41d t, cv::Matx34d outT); 
    void getTransformationMatrix(Eigen::Matrix3d R, Eigen::Vector4d t, Eigen::Matrix<double,3,4> outT); 


    // Eigen::Matrix3d findFundamentalMatrix(const std::vector<std::vector<cv::Point2f>> &matched_points); 
    // Eigen::Matrix3d findEssentialMatrix(const Eigen::Matrix3d &intrinsicMatrix, const Eigen::Matrix3d &fundamentalMatrix); 
    //  void decomposeProjectionMatrix(const Eigen::Matrix<double,3,4> &projectionMatrix, 
    //                                const Eigen::Matrix3d &K, 
    //                                const Eigen::Matrix3d &R, 
    //                                const Eigen::Vector3d &t); 



}