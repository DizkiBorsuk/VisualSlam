#pragma once 
#include "common_includes.hpp"
#include "camera.hpp"
#include "map.hpp"
#include "frame.hpp"

namespace mrVSLAM
{
    class LocalMapping
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        LocalMapping() noexcept; 
        void setLocalMapping(std::shared_ptr<Map> in_map, std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right); 
        void runLocalMapping(); 
        
        void updateMap(); 
        
        void localBundleAdjustment(std::unordered_map<unsigned int, std::shared_ptr<Frame>> keyframes,  std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> mappoints); 

        void stopLocalMapping(); 
        void endLocalMappingThread(); 

    private: 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Camera> camera_left = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr;

        std::thread l_mapping_thread; 
        std::mutex l_mapping_mutex; 
        std::condition_variable map_update_var; //* The condition_variable class is a synchronization primitive used with a std::mutex to block one or more threads until another thread both modifies a shared variable (the condition) and notifies the condition_variable. 
        std::atomic<bool> l_mapping_running; //variable allowing to check if local mapping optimization is in progress
    }; 
}