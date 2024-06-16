/**
 * @file common_includes.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-06
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once 

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <type_traits>
#include <exception>
		
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>

#include <fmt/core.h>
#include <fmt/color.h>

enum class TrackingStatus { INITING, TRACKING, LOST };
enum class DetectorType {GFTT = 0, ORB, SIFT, SUPER_POINT}; 
enum class SLAM_TYPE{STEREO = 0, MONO = 1}; 

namespace mrVSLAM
{   
    /**
     * @brief struct for storing algorithm results 
     */
    struct ResultStruct
    {
        bool finished = false; 
        unsigned int sequence; 
        SLAM_TYPE tracking_type; 
        DetectorType detector; 
        unsigned int num_of_features; 

        float mean_error; 
        float percent_error; 
        float mean_error_x; 
        float mean_error_y; 
        float mean_error_z; 
        float max_error_x;
        float max_error_y; 
        float max_error_z; 
        float min_error_x;
        float min_error_y; 
        float min_error_z; 

        double mean_time;
        double min_time;  
        double max_time; 
    };
}

using namespace std::chrono_literals; // for using ms 