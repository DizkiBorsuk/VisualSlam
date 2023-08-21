#pragma once 
#include "common_includes.hpp"
#include "read_dataset.hpp"

namespace mrVSLAM
{
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

        cv::Ptr<cv::GGFTTDetector>

        KITTI_Dataset dataset; 

        bool initialization_succes = false; 
        std::shared_ptr<Map> ptr_to_map = nullptr; 
        std::shared_ptr<Visuzalization> ptr_to_visualization = nullptr; 
        //std::shared_ptr<> ptr_ = nullptr; 


    };
}

 