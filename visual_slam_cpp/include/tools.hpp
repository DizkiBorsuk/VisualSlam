#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    Eigen::Matrix3d findFundamentalMatrix(const std::vector<std::vector<cv::Point2f>> &matched_points); 
    Eigen::Matrix3d findEssentialMatrix(const Eigen::Matrix3d &intrinsicMatrix, const Eigen::Matrix3d &fundamentalMatrix); 




}