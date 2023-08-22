#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "backend_optimization.hpp"

namespace mrVSLAM
{
    enum class STATUS {INITIALIZATION, TRACKING, LOST}; // status of tracking 

    class Tracking
    {
    public: 

    private: 

        STATUS tracking_status; 
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptor; 

        //### function members ##3// 
        Tracking(); 

        bool addFrame(std::shared_ptr<Frame> frame_to_add); 
        bool track(); 

    }; 
}