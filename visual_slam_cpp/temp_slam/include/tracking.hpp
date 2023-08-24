#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "visualizer.hpp"


namespace mrVSLAM
{
    class Backend; 

    enum class STATUS {INITIALIZATION, TRACKING, LOST}; // status of tracking 

    class Tracking
    {
    public: 
        //### function members ###// 
        Tracking(); 
        void addFrameAndTrack(std::shared_ptr<Frame> frame_to_add); // main function of tracking

        // 3 state functions 
        bool stereoInitialize(); 
        void track(); 
        void RestartTracking(); 

        //feature detection functions 
        unsigned int detectFeatures(); // detect features in frame left img and return number of found points
        unsigned int findCorrFeatures(); 

        // 
        bool initializeMap(); 

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