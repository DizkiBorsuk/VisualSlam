#include "../include/local_mapping.hpp"
#include "../include/map.hpp"
#include "../include/frame.hpp"
#include "../include/graph_structure.hpp"

namespace mrVSLAM
{
    LocalMapping::LocalMapping() noexcept
    {
        // std::bind generates a call wrapper for function, calling wrapper is like invokeing function //works like function pointer
        l_mapping_thread = std::thread(std::bind(&LocalMapping::runLocalMapping, this)); // start bakcend thread
        std::cout << "started optimization thread \n"; 
        l_mapping_running.store(true);
    }

    void LocalMapping::setLocalMapping(std::shared_ptr<Map> in_map, std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right)
    {
        map = in_map; 
        camera_left = in_camera_left; 
        camera_right = in_camera_right; 
        std::cout << "backend pointers to map and cameras set \n";   
    }

    void LocalMapping::endLocalMappingThread()
    {
        l_mapping_thread.join(); // join to main thread
        map_update_var.notify_one(); 
        l_mapping_running.store(false); 
        std::cout << "end of local mapping optimization thread \n"; 
    }

    void LocalMapping::runLocalMapping()
    {
        while(l_mapping_running.load() == true)
        {
            std::unique_lock<std::mutex> lock(l_mapping_mutex); 
            map_update_var.wait(lock);  // thread waits for new keyframe and mappoints 

            std::unordered_map<unsigned int, std::shared_ptr<Frame>> active_keyframes = map->getEnabledKeyframes(); 
            std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> active_mappoints = map->getEnabledMappoints(); 

            localBundleAdjustment(active_keyframes, active_mappoints); 

        }
    }


    void LocalMapping::updateMap()
    {
       std::lock_guard<std::mutex> lock(l_mapping_mutex); 
       map_update_var.notify_one(); //https://en.cppreference.com/w/cpp/thread/condition_variable/notify_one
    }

    void LocalMapping::stopLocalMapping()
    {
        
    }

    void LocalMapping::localBundleAdjustment(std::unordered_map<unsigned int, std::shared_ptr<Frame>> keyframes,  std::unordered_map<unsigned int, std::shared_ptr<MapPoint>> mappoints)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; 
    }
    
}