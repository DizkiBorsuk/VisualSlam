#pragma once 
#include "common_includes.hpp"

namespace mrVSLAM
{
    class Backend
    {
    public: 

        Backend(); 

        void setMap(); 
        void updateMap(); 
        
        void optimize(); 

    private: 

        std::shared_ptr<Map> ptr_to_map; 
        
        std::thread backend_thread; 
        std::mutex backend_mutex; 
        std::condition_variable map_update; // The condition_variable class is a synchronization primitive used with a std::mutex to block one or more threads until another thread both modifies a shared variable (the condition) and notifies the condition_variable. 
        
    
    }; 
}