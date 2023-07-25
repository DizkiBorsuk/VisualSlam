#pragma once
#include "system.hpp"
#include "readDataset.hpp"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h> 
#include "matplotlibcpp.h"


namespace mrVSLAM
{
    inline void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, 
                   std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& poses, const int num_of_frames); 
                   
    inline void plotPoses(std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>& gt_poses, const int num_of_frames); 

    void plotPerformance(std::vector<int> loopTimes); 
}