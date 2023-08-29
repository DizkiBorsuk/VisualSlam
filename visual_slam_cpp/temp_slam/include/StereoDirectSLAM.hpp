#pragma once 
#include "common_includes.hpp"
#include "read_dataset.hpp"
#include "tracking.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "backend_optimization.hpp"
#include "visualizer.hpp"
#include "camera.hpp"

namespace mrVSLAM
{

    class StereoDirectSLAM
    {
    public: 
        std::vector<int> performance; 
        std::vector<Eigen::Matrix<double, 3,4>> trajectory; 
        Camera camera_left; 
        Camera camera_right; 

        StereoDirectSLAM(std::string sequence_number); 
        ~StereoDirectSLAM(); 

        int Run(); // main execution loop  

    private: 
        int frame_counter = 0; 
        int fps = 0, loopStart = 0, loopEnd = 0; 

        bool initialization_succes = false; 

        //KITTI_Dataset dataset; 
        // 
        std::shared_ptr<Tracking> tracking = nullptr; 
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Backend> backend = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<KITTI_Dataset> dataset = nullptr; 
        //std::shared_ptr<> ptr_ = nullptr; 


    };
}

 