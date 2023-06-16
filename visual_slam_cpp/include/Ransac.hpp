#pragma once 
#include "system.hpp"

namespace mrVSLAM
{

    std::vector<std::vector<cv::Point2f>> ransac(std::vector<std::vector<cv::Point2f>> data, 
                                                const int minSamples, 
                                                const float residualTreshold, 
                                                const int maxTrials = 100)
    {
        //https://github.com/scikit-image/scikit-image/blob/main/skimage/measure/fit.py

        int best_inlier_num = 0; 
        std::vector<std::vector<cv::Point2f>> inliers; 
        std::vector<cv::Point2f> best_inliers; 


        return inliers; 
    }




}
