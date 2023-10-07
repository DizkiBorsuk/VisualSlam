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

        auto linearSolver = std::make_unique<LinearSolverType>(); 
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::move(linearSolver))); 
        g2o::SparseOptimizer optimizer; 
        optimizer.setAlgorithm(solver); 

        std::map<unsigned long int, Pose3DVertex*> pose_verticies; 
        unsigned int max_keyframe_id = 0; //? maybe change to long 

        for(auto &kf_map_entry : keyframes)
        {
            auto keyframe = kf_map_entry.second; 
            Pose3DVertex* pose_vertex = new Pose3DVertex; 
            pose_vertex->setId(keyframe->keyframe_id); //set pose vertex id to keyframe id 
            pose_vertex->setEstimate(keyframe->getSophusFramePose()); // set initiall guess for pose 
            optimizer.addVertex(pose_vertex); 
            if(keyframe->keyframe_id > max_keyframe_id)
            {
                max_keyframe_id = keyframe->keyframe_id; 
            }
            pose_verticies.insert({keyframe->keyframe_id, pose_vertex});
        }

        //set verticies for mappoints 
        std::map<unsigned int, PointVertex*> mapppoint_verticies; 
        std::map<PointPoseEdge*, std::shared_ptr<Feature>> observation_edges; // map that assigns edge to the feature that it represents 

        //get camera intrinsics and extrinsics 
        Eigen::Matrix3d K = camera_left->K_eigen; 
        Sophus::SE3 left_camera_Rt = Sophus::SE3d(camera_left->Rt);
        Sophus::SE3 right_camera_Rt = Sophus::SE3d(camera_right->Rt);

        unsigned int id = 1; 

        for(auto &observed_point : mappoints) // observed point is hash map entry of mappoints 
        {
            unsigned int point_id = observed_point.second->id; 
            auto observed_point_features = observed_point.second->getFeatures(); 

            for(auto &feature : observed_point_features)
            {
                if(feature.lock() == nullptr)
                {
                    continue;
                }
                
            }

        }

    }

  //Enf of local mapping   
}