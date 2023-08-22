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
        //### function members ###// 
        Tracking(); 

        bool track(); 
        bool addFrame(std::shared_ptr<Frame> frame_to_add); 

    private: 

        STATUS tracking_status = STATUS::INITIALIZATION; 
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptor; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr;

        unsigned int num_of_features = 300; 
        unsigned int num_of_features_init = 100; //i guess? 
        unsigned int num_of_features_for_keyframe = 50; //check in orbslam 



        

    }; 
}