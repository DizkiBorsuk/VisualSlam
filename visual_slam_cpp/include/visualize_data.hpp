#pragma once
#include "system.hpp"
#include "readDataset.hpp"

namespace plt = matplotlibcpp;


namespace mrVSLAM
{
    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& poses, const int num_of_frames); 
                   
    void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames); 

    void plotPoses3d(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames)


}