/**
 * @file local_mapping.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/local_mapping.hpp" 
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/camera.hpp"
#include "mrVSLAM/mappoint.hpp"
#include "mrVSLAM/tools.hpp"


namespace mrVSLAM
{
    //! done
    LocalMapping::LocalMapping()
    {
        // set atomics 
        localMappingRunning.store(true); 
        pauseRequest.store(false); 
        threadPaused.store(false); 

        // start thread 
        local_mapping_thread = std::thread(std::bind(&LocalMapping::runLocalMappingThread, this)); 
        fmt::print(fg(fmt::color::aqua), "local mapping thread started \n"); 
    }

    void LocalMapping::updateMap()
    {
        std::unique_lock<std::mutex> lock(local_mapping_mutex);
        map_update.notify_one(); 
    }

    void LocalMapping::stop()
    {
        localMappingRunning.store(false); 
        map_update.notify_one();
        fmt::print("local mapping, after unlcoking in stop \n"); 
        local_mapping_thread.join();
        fmt::print(fg(fmt::color::indian_red), "local mapping thread closed \n"); 
    }

    void LocalMapping::requestPause()
    {
        pauseRequest.store(true); 
        fmt::print(fg(fmt::color::yellow), "local mapping thread paused \n"); 
    }

    bool LocalMapping::confirmPause()
    {
        return (pauseRequest.load() && threadPaused.load());   
    }

    void LocalMapping::resume()
    {
        pauseRequest.store(false); 
        threadPaused.store(false); 
        fmt::print(fg(fmt::color::yellow), "local mapping thread restarted \n"); 
    }

//* ------ Main part ------- *// 

    void LocalMapping::runLocalMappingThread()
    {
        while(localMappingRunning.load())
        {   
            while(pauseRequest.load())
            {
                threadPaused.store(true); 
                std::this_thread::sleep_for(100us); 
            }

            threadPaused.store(false); 

            auto beginT = std::chrono::steady_clock::now();

            std::unique_lock<std::mutex> lock(local_mapping_mutex);
            map_update.wait(lock); 

            Map::KeyframesType active_kfs = map->getActiveKeyframes();
            Map::LandmarksType active_landmarks = map->getActiveMappoints();

            localBundleAndjustment(active_kfs, active_landmarks); 

            auto endT = std::chrono::steady_clock::now();
            auto elapsedT = std::chrono::duration_cast<std::chrono::milliseconds>(endT - beginT);
            fmt::print("Local mapping: loop time = {} \n", elapsedT.count()); 
        }
    }
    
    void LocalMapping::localBundleAndjustment(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks)
    {
        // setup g2o
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;

        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
            vertex_pose->setId(kf->kf_id);
            vertex_pose->setEstimate(kf->getPose());
            optimizer.addVertex(vertex_pose);

            if (kf->kf_id > max_kf_id) 
            {
                max_kf_id = kf->kf_id;
            }

            vertices.insert({kf->kf_id, vertex_pose});
        }

        std::map<unsigned long, VertexXYZ *> vertices_landmarks;
        Eigen::Matrix3d K = camera_left->getK();
        Sophus::SE3d left_ext = camera_left->getPose();
        Sophus::SE3d right_ext = camera_right->getPose();

        // edges
        int index = 1;
        double chi2_th = 5.991;  
        //double chi2_th = 7.815; 
        std::map<EdgeProjection *, std::shared_ptr<Feature>> edges_and_features;

        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier) 
                continue;

            unsigned int landmark_id = landmark.second->id;
            auto observations = landmark.second->getObservations();

            for (auto &obs : observations) 
            {
                if (obs.lock() == nullptr) 
                    continue;
                auto feat = obs.lock();
                
                if (feat->is_outlier || feat->keyframe.lock() == nullptr) 
                    continue;

                auto frame = feat->keyframe.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_img) {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                if (!vertices_landmarks.contains(landmark_id))  //vertices_landmarks.find(landmark_id) == vertices_landmarks.end()
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->getPointPosition());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                if (vertices.contains(frame->kf_id) && vertices_landmarks.contains(landmark_id))  //vertices.find(frame->keyframe_id) != vertices.end() && vertices_landmarks.find(landmark_id) != vertices_landmarks.end()
                {
                        edge->setId(index);
                        edge->setVertex(0, vertices.at(frame->kf_id));    // pose
                        edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                        edge->setMeasurement(convertToVec(feat->positionOnImg.pt));
                        edge->setInformation(Eigen::Matrix2d::Identity());
                        auto rk = new g2o::RobustKernelHuber();
                        rk->setDelta(chi2_th);
                        edge->setRobustKernel(rk);
                        edges_and_features.insert({edge, feat});
                        optimizer.addEdge(edge);
                        index++;
                } else {
                    delete edge; 
                }
            }
        }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int outlier = 0, inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            outlier = 0;
            inlier = 0;
    
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    outlier++;
                } else {
                    inlier++;
                }
            }
            double inlier_ratio = inlier / double(inlier + outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier = true;
                // remove the observation
                auto mp = ef.second->map_point.lock(); 
                mp->removeObservation(ef.second);
                mp->removeActiveObservation(ef.second);

                if(mp->getObservations().empty()) {
                    mp->is_outlier=true; 
                    map->addOutlierPoint(mp->id); 
                }
            } else {
                ef.second->is_outlier = false;
            }
        }

        fmt::print(fg(fmt::color::blue), "Outlier/Inlier in optimization: {}/{} \n", outlier, inlier);

        // Set pose and lanrmark position
        for (auto &v : vertices) 
        {
            keyframes.at(v.first)->setPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) 
        {
            landmarks.at(v.first)->setPosition(v.second->estimate());
        }

        map->removeOutliers(); 

    }

} //! end of namespace