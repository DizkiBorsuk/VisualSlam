/**
 * @file local_mapping.hpp
 * @author mrostocki 
 * @brief local mapping (local map optimization) class/thread 
 * @version 0.1
 * @date 2024-03-06
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 
#include "mrVSLAM/graph_structures.hpp"
#include "mrVSLAM/map.hpp"

namespace mrVSLAM
{   
    class Camera; 
    class Visualizer; 
    class Map; 
    class Frame; 
    class LoopCloser; 

    /**
     * @brief LocalMapping module/thread takes care of local map optimization. 
     * It performs bundle adjustment on small set of newest keyframes and associated map points (local map). 
     */
    class LocalMapping
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  
        
        /**
         * @brief Construct a new Local Mapping object
         */
        LocalMapping(); 
        void setLocalMapping(std::shared_ptr<Map> map_ptr, std::shared_ptr<Camera> l_cam_ptr, std::shared_ptr<Camera> r_cam_ptr)
        {
            map = map_ptr; 
            camera_left = l_cam_ptr; 
            camera_right = r_cam_ptr; 
        }

        void updateMap(); 

        // functions for stoping and resuming thread 
        /**
         * @brief top local mapping and destroy thread 
         */
        void stop(); 
        
        /**
         * @brief pause thread by changing std::atomic<bool> pauseRequest to true 
         */
        void requestPause(); 
        
        /**
         * @brief allows to check if local mappping thread is running or is it in pause
         * @return true - thread is currently paused
         * @return false - guess what, it's not 
         */
        bool confirmPause(); 
        void resume(); 
    
    private: 

        //bool checkForNewKeyframes(); 
        //void processNewKeyframes(); 
        void runLocalMappingThread(); 
        void localBundleAndjustment(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks); 


    private: 

        std::shared_ptr<Camera> camera_left = nullptr, camera_right = nullptr; 
        std::shared_ptr<Map> map = nullptr; 
        
        std::thread local_mapping_thread; 
        std::atomic<bool> localMappingRunning; 
        std::atomic<bool> pauseRequest; 
        std::atomic<bool> threadPaused; 

        std::mutex local_mapping_mutex; 
        std::mutex stop_mutex; 
        std::mutex kf_mutex; 
        std::condition_variable map_update; 

    }; 
} //! end of namespace 
