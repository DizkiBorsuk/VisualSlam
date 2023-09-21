#pragma once 
#include "common_includes.hpp"
#include "camera.hpp"
#include "map.hpp"
#include "frame.hpp"

namespace mrVSLAM
{
    class Backend
    {
    public: 

        Backend() noexcept; 
        void setBackend(std::shared_ptr<Map> in_map, std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right); 
        void runBackend(); 
        
        void updateMap(); 
        
        void optimize(); 

        void bundleAdjustment(const std::vector<std::shared_ptr<Frame>> allKeyFrames, const std::vector<std::shared_ptr<MapPoint>>, unsigned int iter); 

        void endBackendThread(); 

    private: 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Camera> camera_left = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr;

        std::thread backend_thread; 
        std::mutex backend_mutex; 
        std::condition_variable map_update; //* The condition_variable class is a synchronization primitive used with a std::mutex to block one or more threads until another thread both modifies a shared variable (the condition) and notifies the condition_variable. 
        
    
    }; 
}