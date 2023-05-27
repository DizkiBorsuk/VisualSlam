#pragma once 
#include "system.hpp"


namespace mrVSLAM
{
    class SLAM
    {
        public:
        int f_counter = 0; 
        int executeMonoSLAM(const std::string& imgs_path); 
        int executeGPUMonoSLAM(const std::string& imgs_path); 
        void executeStereoSLAM(const std::string& left_camera_path, std::string& right_camera_path, bool gpu = true);

    }; 


}


