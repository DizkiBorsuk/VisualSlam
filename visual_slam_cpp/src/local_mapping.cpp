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
        map_update_var.notify_one(); 
        l_mapping_running.store(false); 
        l_mapping_thread.join(); // join to main thread
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
        double chi_squared_treshold = 5.991; // {9.210,7.378,5.991,5.991};

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

        unsigned int edge_id = 1; 

        for(auto &observed_point : mappoints) // observed point is hash map entry of mappoints 
        {
            unsigned int point_id = observed_point.second->id; 
            auto observed_point_features = observed_point.second->getFeatures(); 

            for(auto &observation : observed_point_features)
            {
                auto feature = observation.lock(); // convert weak ptr to shared ptr 
                
                if( (feature==nullptr) || (feature->outlier==true) || (feature->frame.lock()==nullptr)) //?hmmm smth didn't work here but now it does? 
                {
                    continue;
                }

                auto frame = feature->frame.lock(); 

                PointPoseEdge* edge = nullptr; 

                if(feature->on_leftImg == true) //if feature is on left img give edge left camera Rt matrix 
                {
                    edge = new PointPoseEdge(K, left_camera_Rt); 
                } 
                else  //else give it right camera Rt
                {
                    edge = new PointPoseEdge(K, right_camera_Rt); 
                }

                if( !mapppoint_verticies.contains(point_id)) //check if verticies contain 
                {
                    PointVertex* p_vertex = new PointVertex; 
                    p_vertex->setEstimate(observed_point.second->getPosition()); 
                    p_vertex->setId(point_id+max_keyframe_id+1); 
                    p_vertex->setMarginalized(true);
                    mapppoint_verticies.insert({point_id, p_vertex}); 
                    optimizer.addVertex(p_vertex); 
                }

                if(pose_verticies.contains(frame->keyframe_id) && mapppoint_verticies.contains(point_id))
                {
                    edge->setId(edge_id); 
                    edge->setVertex(0, pose_verticies.at(frame->keyframe_id)); // set pose Vertex of the edge 
                    edge->setVertex(1,mapppoint_verticies.at(point_id)); //set mappoint node of edge 
                    Eigen::Vector2d feature_pt;
                    feature_pt << feature->featurePoint_position.pt.x, feature->featurePoint_position.pt.y; 
                    edge->setMeasurement(feature_pt); 
                    edge->setInformation(Eigen::Matrix2d::Identity()); 

                    auto kernel = new g2o::RobustKernelHuber(); 
                    kernel->setDelta(chi_squared_treshold); 
                    edge->setRobustKernel(kernel); 

                    observation_edges.insert({edge, feature}); 
                    optimizer.addEdge(edge); 

                    edge_id++; 
                }
                else
                {
                    delete edge; 
                }

            }
        }

        optimizer.initializeOptimization(); //don't have fixed control points so i don't need .fixed() //? why is it used in orbslam thou
        optimizer.optimize(10); //! set to 10, will change later 
    
        //optimize based on inliers/outliers ratio 
        unsigned int outliers = 0;
        unsigned int inliers = 0;

        int iter = 0; 
        while(iter < 5)
        {
            for(auto &obs_edge : observation_edges)
            {
                if(obs_edge.first->chi2() > chi_squared_treshold)
                {
                    outliers++; 
                } 
                else 
                {
                    inliers++; 
                }
            }
            
            double good_bad_ratio = inliers/ double(inliers + outliers); 

            if(good_bad_ratio > 0.5)
            {
                break; 
            } 
            else 
            {
                chi_squared_treshold*=2; 
                iter++; 
            }    
        }

        //outlier rejection 
        for(auto &obs_edge : observation_edges)
        {
            if(obs_edge.first->chi2() > chi_squared_treshold)
            {
                obs_edge.second->outlier=true; 
                obs_edge.second->map_point.lock()->removeFeature(obs_edge.second); 
            } 
            else 
            {
                obs_edge.second->outlier=false; //not sure if needed, better safe than sorry 
            }
        }

        for(auto &p_vertex : pose_verticies)
        {
            keyframes.at(p_vertex.first)->SetFramePose(p_vertex.second->estimate()); 
        }

        for(auto &mappoint_vertex : mapppoint_verticies)
        {
            mappoints.at(mappoint_vertex.first)->setPosition(mappoint_vertex.second->estimate()); 
        }
        
        std::cout << "ended optimizing local map \n"; 
    }

  //Enf of local mapping   
}