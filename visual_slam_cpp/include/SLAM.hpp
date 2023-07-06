#pragma once 
#include "system.hpp"
#include "camera.hpp"
#include "readDataset.hpp"

namesepace mrVSLAM
{
    

    class SLAM
    {
    private: 

    public: 
        enum SlamType {feature, direct}
        
        SLAM(SlamType, std::string &dataset_path); 
        int runSLAM(); 
        int runGpuSLAM();  
    }; 

}



