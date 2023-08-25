#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"
#include "map.hpp"

/*
    Tracking is responsible for calculating frame's pose based on img/imgs, feature detection, map initalization 
*/

namespace mrVSLAM
{
    class Backend; 
    class Visualizer; 

    enum class STATUS {INITIALIZATION, TRACKING, LOST}; // status of tracking 

    class Tracking
    {
    public: 
        //### function members ###// 
        Tracking(); 
        void setTracking(std::shared_ptr<Map> in_map, std::shared_ptr<Visualizer> in_visualizer, std::shared_ptr<Backend> in_backend); 
        void addFrameAndTrack(std::shared_ptr<Frame> frame_to_add); // main function of tracking

        // 3 state functions 
        bool initialize(); // initialization for monocular case 
        bool stereoInitialize(); // initialization for stereo case 
        void track(); 
        void restartTracking(); 

        //feature detection functions 
        unsigned int detectFeatures(); // detect features in frame left img and return number of found points
        unsigned int findCorrFeatures(); 

        // 
        bool initializeMap(); 

    private: 

        STATUS tracking_status = STATUS::INITIALIZATION; 
        
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptorExtractor; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr;
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<Backend> backend = nullptr; 

        unsigned int num_of_features = 300; 
        unsigned int num_of_features_init = 100; // numbers of features needed to be found in both imgs for initalization success 
        unsigned int num_of_features_for_keyframe = 50; //? check in orbslam 

        unsigned int inliers = 0; 

        cv::Matx44d transformationMatrix; 


    }; 
}