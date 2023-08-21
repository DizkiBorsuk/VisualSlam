#pragma once 
#include "common_includes.hpp"
#include "read_dataset.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "backend_optimization.hpp"
#include "visualizer.hpp"

namespace mrVSLAM
{
    enum class STATUS {INITIALIZATION, TRACKING, LOST}; 

    class StereoDirectSLAM
    {
    public: 

        StereoDirectSLAM(std::string sequence_number); 
        ~StereoDirectSLAM(); 
        void Run(); 

    private: 

        unsigned int num_of_features = 300; 
        unsigned int num_of_features_init = 100; //i guess? 
        unsigned int num_of_features_for_keyframe = 50; //check in orbslam 
        bool initialization_succes = false; 

        cv::Ptr<cv::GFTTDetector> featureDetector; 

        KITTI_Dataset dataset; 

        
        std::shared_ptr<Map> ptr_to_map = nullptr; 
        std::shared_ptr<Visualizer> ptr_to_visualizer= nullptr; 
        //std::shared_ptr<> ptr_ = nullptr; 


    };
}

 