#pragma once 
#include "common_includes.hpp"
#include "matplotlibcpp.h"

namespace mrVSLAM
{
    void getTransformationMatrix(const cv::Matx33d &R,const cv::Matx31d &t, cv::Matx44d &outT); 
    void getTransformationMatrix(const Eigen::Matrix3d &R, const Eigen::Vector4d &t, Eigen::Matrix<double,4,4> &outT); 

    //### visualization tools ###// 

    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& poses, const int num_of_frames); 
                   
    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames); 
    void plotPoses(std::vector<cv::Matx44d>& poses, std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames);

    void plotPerformance(std::vector<int> loopTimes); 


    // ######## 
    inline bool triangulate(const std::vector<Eigen::Matrix<double,2,1>> &points,const Eigen::Matrix<double,3,4> &P_matrix, std::array<float,3> &point_pos); //!change pos to Eigen
}