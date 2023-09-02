#pragma once 
#include "common_includes.hpp"
#include "map.hpp"
#include "frame.hpp"

namespace mrVSLAM
{
    class Backend
    {
    public: 

        Backend() noexcept; 

        void setMapPtr();  
        void updateMap(); 
        
        void optimize(); 

        void bundleAdjustment(const std::vector<std::shared_ptr<Frame>> allKeyFrames, const std::vector<std::shared_ptr<MapPoint>>, unsigned int iter); 

    private: 

        std::shared_ptr<Map> map = nullptr; 
        
        std::thread backend_thread; 
        std::mutex backend_mutex; 
        std::condition_variable map_update; //* The condition_variable class is a synchronization primitive used with a std::mutex to block one or more threads until another thread both modifies a shared variable (the condition) and notifies the condition_variable. 
        
    
    }; 
}