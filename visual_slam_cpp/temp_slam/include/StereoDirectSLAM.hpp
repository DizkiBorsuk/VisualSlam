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
        bool initialization_succes = false; 

        cv::Ptr<cv::GGFTTDetector>

        KITTI_Dataset dataset; 

        
        std::shared_ptr<Map> ptr_to_map = nullptr; 
        std::shared_ptr<Visuzalizer> ptr_to_visuzalizer= nullptr; 
        //std::shared_ptr<> ptr_ = nullptr; 


    };
}

 