#pragma once 
#include "system.hpp"


namespace mrVSLAM
{
    class SLAM
    {
        public:
        int f_counter = 0; 
        int executeMonoSLAM(std::string& imgs_path); 
        int executeGPUMonoSLAM(std::string& imgs_path); 
        void executeStereoSLAM(std::string& left_camera_path, std::string& right_camera_path, bool gpu = true);

    }; 


}


