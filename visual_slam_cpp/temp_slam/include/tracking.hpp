#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"
#include "map.hpp"


namespace mrVSLAM
{
    class Visualizer; 
    class Backend; 

    enum class STATUS {INITIALIZATION, TRACKING, LOST}; // status of tracking 

    class Tracking
    {
    public: 
        //### function members ###// 
        Tracking(); 
        void addFrameAndTrack(std::shared_ptr<Frame> frame_to_add); // main function of tracking

        void stereoInitialize(); 
        void track(); 
        unsigned int detectFeatures(); // detect features in frame left img and return number of found points
        

    private: 

        STATUS tracking_status = STATUS::INITIALIZATION; 
        
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptor; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr;
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 

        unsigned int num_of_features = 300; 
        unsigned int num_of_features_init = 100; //i guess? 
        unsigned int num_of_features_for_keyframe = 50; //check in orbslam 

        cv::Matx44d transformationMatrix; 



        

    }; 
}