#include "myslam/loop_closing.hpp"
#include "myslam/frame.hpp"
#include "myslam/g2o_types.hpp"
#include "myslam/map.hpp"
#include "myslam/camera.hpp"
#include "myslam/tools.hpp"
#include <g2o/types/slam3d/types_slam3d.h>

namespace myslam
{
    LoopClosing::LoopClosing(std::shared_ptr<DBoW3::Vocabulary> vocab)
    {
        database = DBoW3::Database(*vocab, false, 0); 
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 
        loop_closer_running.store(true); 
        loop_closer_thread = std::thread(std::bind(&LoopClosing::runLoopCloser, this)); 
    }

    void LoopClosing::addCurrentKeyframe(std::shared_ptr<Frame> new_keyframe)
    {
        /*
        adds current keyframe to keyframe database, then it is compared with every other keyframe.
        */
        std::unique_lock<std::mutex> lock(loop_closer_mutex); 
        current_frame = new_keyframe; 
        map_update.notify_one();
    }

    void LoopClosing::letLoopCLoser()
    {
        std::unique_lock<std::mutex> lock(loop_closer_mutex); 
        local_mapping_update.notify_one();
    }

    void LoopClosing::end()
    {
        loop_closer_running.store(false); 
        map_update.notify_one(); 
        loop_closer_thread.join(); 
    }

    void LoopClosing::runLoopCloser()
    {
        while(loop_closer_running.load())
        {
            std::unique_lock<std::mutex> lock(loop_closer_mutex);
            map_update.wait(lock);

            cv::Mat descriptors; 
            for(auto &feature : current_frame->features_left_)
            {
                descriptors.push_back(feature->descriptor); 
            }

            
            vocabulary->transform(descriptors, current_frame->bow_vector); 

            database.add(current_frame->bow_vector); 
            DBoW3::QueryResults similarity; 

            unsigned int loop_candidate_id = 0; 

            if(database.size()>25)
            {
                database.query(current_frame->bow_vector, similarity, 10); 

                for(std::size_t s = 0; s < similarity.size(); s++)
                {
                    std::cout << "miara podobieństwa = " <<  similarity.at(s) << "\n";  
      
                    //i only have to check score at second entry because scores are sorted and first one is current frame itself
                    if(similarity.at(s).Score > 0.025 && similarity.at(s).Score < 0.95) //0.06/7 for gftt and 0.03 for orb
                    {
                        std::cout << "for keyframe"  << current_frame->keyframe_id << " found a loop candidate with id = " << similarity.at(s).Id << "\n"; 
                        std::cout << " score is " << similarity.at(s).Score << "\n"; 
                        loop_candidate_id = similarity.at(s).Id; // id of frame that current frame is similar to 

                        auto matched_kf = map->getKeyFrameById(loop_candidate_id); 

                        if(current_frame->keyframe_id - matched_kf->keyframe_id > 20)
                        {
                            std::array<std::shared_ptr<Frame>,2> pair;
                            pair.at(0) = current_frame;
                            pair.at(1) =  matched_kf; 
                            keyframe_pairs.emplace_back(pair); 

                            //! match keypoints and calculate transformation 
                            // matchLoopFeatures(matched_kf); 

                            //! wait for local mapping 
                            local_mapping_update.wait(lock); 
                                globalBundleAdjustment(current_frame->keyframe_id, loop_candidate_id); 
                            break; 
                        }
                    }
                }
            }
                    
            std::cout << "database size " << database.size() << "\n"; 
            std::cout << "kf pair size " << keyframe_pairs.size() << "\n"; 
        }
    }


    bool LoopClosing::matchLoopFeatures(std::shared_ptr<Frame> loop_candidate_kf)
    {
        /*
        match features and compute pose transformation 
        */
        std::vector<cv::DMatch> matches; 
        cv::Mat descriptors_current, descriptors_loop_candidate; 

        for(size_t i=0; i < current_frame->features_left_.size(); i++)
        {
            descriptors_current.push_back(current_frame->features_left_[i]->descriptor); 
        }

        for(size_t i=0; i < loop_candidate_kf->features_left_.size(); i++)
        {
            descriptors_loop_candidate.push_back(loop_candidate_kf->features_left_[i]->descriptor); 
        }

        matcher->match(descriptors_current, descriptors_loop_candidate, matches); 

        double min_dist=10000, max_dist=0;
        for (size_t i = 0; i < matches.size(); i++ )
        {
            double dist = matches[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        unsigned int found_matches; 
        std::vector<std::array<int, 2>> kp_pairs_ids; 
        for (std::size_t i = 0; i < matches.size(); i++ )
        {
            if(matches[i].distance <= std::max( 2*min_dist, 30.0 ))
            {
                found_matches++; 
                std::array<cv::KeyPoint, 2> good_matches; 
                kp_pairs_ids.emplace_back((matches[i].queryIdx, matches[i].trainIdx));                                                                     
            }
        }
        std::cout << "in loop closing: found " << found_matches << "good matches \n"; 

        if(found_matches < 10)
        {
            return false; 
        }

        //! compute transformation 

        for(auto &kfp : kp_pairs_ids)
        {
            auto mappoint = loop_candidate_kf->features_left_[kfp.at(1)]->map_point_.lock(); 
            if(mappoint)
            {

            }
        }


        return true; 
    }

    void LoopClosing::PoseGraph(std::shared_ptr<Frame> loop_candidate_kf)
    {
        Map::KeyframesType keyframes = map->GetAllKeyFrames();

        //set g2o bullshit 
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver); 

        std::map<unsigned long, VertexPose *> vertices_kf; 
        for(auto &keyframe: keyframes)
        {
            unsigned long kfId = keyframe.first;
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id);
            vertex_pose->setEstimate(kf->getPose());
            vertex_pose->setMarginalized(false);

            auto mapActiveKFs = map->GetActiveKeyFrames();
            // active KFs, loop KF, initial KF are fixed
            if( mapActiveKFs.find(kfId) != mapActiveKFs.end() || (kfId == loop_candidate_kf->keyframe_id) || kfId == 0)
            {
                vertex_pose->setFixed(true);
            }

            optimizer.addVertex(vertex_pose);
            vertices_kf.insert({kf->keyframe_id, vertex_pose});
        }

        int index = 0;
        std::map<int, EdgePoseGraph *> vEdges;
        for(int i = 1; i < keyframes.size(); i++)
        {
            unsigned long kfId = keyframes[i]-> keyframe_id; 
            assert(vertices_kf.contains(kfId));
            auto kf = keyframes[i]; 

            // edge type 1: edge between two KFs adjacent in time
            auto prev_kf = keyframes[i-1]; 
     
            EdgePoseGraph *edge = new EdgePoseGraph();
            edge->setId(index);
            edge->setVertex(0, vertices_kf.at(kfId));
            edge->setVertex(1, vertices_kf.at(prev_kf->keyframe_id));
            edge->setMeasurement(kf->mRelativePoseToLastKF);
            edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
            optimizer.addEdge(edge);
            vEdges.insert({index, edge});
            index++;
        }   
 //create loop edge
        EdgePoseGraph *edge = new EdgePoseGraph();
        edge->setId(index);
        edge->setVertex(0, vertices_kf.at(current_frame->keyframe_id));
        edge->setVertex(1, vertices_kf.at(loop_candidate_kf->keyframe_id));
        edge->setMeasurement(mRelativePoseToLoopKF);
        edge->setInformation(Eigen::Matrix<double,6,6>::Identity());
        optimizer.addEdge(edge);
        vEdges.insert({index, edge});
        index++;
        
        // do the optimization
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        // correct the KFs' poses
        // correct all mappoints positions according to the KF which first observes it
        { // mutex
            // avoid the conflict between frontend tracking and loopclosing correction
            std::unique_lock<std::mutex> lock(map->public_map_mutex);

            // set the mappoints' positions according to its first observing KF's optimized pose
            auto allMapPoints = map->GetAllMapPoints();
            auto activeMapPoints = map->GetActiveMapPoints();
            
            for(auto iter = activeMapPoints.begin(); iter != activeMapPoints.end(); iter++){
                allMapPoints.erase((*iter).first);
            }
            for(auto &mappoint: allMapPoints)
            {
                std::shared_ptr<MapPoint> mp = mappoint.second;

                assert(!mp->GetObs().empty());

                auto feat = mp->GetObs().front().lock();
                auto observingKF = feat->frame_.lock();

                if(vertices_kf.contains(observingKF->keyframe_id))
                {
                    // NOTICE: this is for the case that one mappoint is inserted into map in frontend thread
                    // but the KF which first observes it hasn't been inserted into map in backend thread
                    continue;
                }
                Eigen::Vector3d posCamera = observingKF->getPose() * mp->Pos();

                Sophus::SE3d T_optimized = vertices_kf.at(observingKF->keyframe_id)->estimate();
                mp->SetPos(T_optimized.inverse() * posCamera);
            }

            // set the KFs' optimized poses
            for (auto &v: vertices_kf) {
                keyframes.at(v.first)->SetPose(v.second->estimate());
            }
            
        } // mutex
    }



    void LoopClosing::globalBundleAdjustment(unsigned int loop_candidate_1_id, unsigned int loop_candidate_2_id)
    {   
        /*
        
        */
        // get all keyframes and landmarks 
        Map::KeyframesType keyframes = map->GetAllKeyFrames();
        Map::LandmarksType landmarks = map->GetAllMapPoints();

        //set g2o bullshit 
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        //map of veticies that represents all camera poses 
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;

        //for every pose/keyframe create vertes representing this pose 
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id);
            
            // if(kf->keyframe_id = loop_candidate_1_id)
            //     vertex_pose->setEstimate(keyframes[loop_candidate_2_id]->getPose());
            // else
            vertex_pose->setEstimate(kf->getPose());

            optimizer.addVertex(vertex_pose);

            if (kf->keyframe_id > max_kf_id) 
            {
                max_kf_id = kf->keyframe_id;
            }

            vertices.insert({kf->keyframe_id, vertex_pose});
        }
        
        //create an edge between two pose vertexes and connect thowse two keyframes that were concluded as the same keyframe
        // optimizer.vertex(loop_candidate_1_id)->setEstimateData(keyframes[loop_candidate_2_id]->getPose()); 
        // vertices[loop_candidate_1_id]->setEstimate(keyframes[loop_candidate_2_id]->getPose()); 
        // vertices[loop_candidate_1_id]->setFixed(true);

        // map of verticies that represents all map points 
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        Eigen::Matrix3d K = camera_left->getK();
        Sophus::SE3d left_ext = camera_left->pose();
        Sophus::SE3d right_ext = camera_right->pose();

        int index = 1;
        double chi2_th = 7.991;  
        //create edges 
        std::map<EdgeProjection *, std::shared_ptr<Feature>> edges_and_features;

        for (auto &landmark : landmarks) 
        {
            if (landmark.second->is_outlier_) 
                continue;

            unsigned int landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();

            for (auto &obs : observations) 
            {
                if (obs.lock() == nullptr) 
                    continue;
                auto feat = obs.lock();
                
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) 
                    continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                if (!vertices_landmarks.contains(landmark_id))  //vertices_landmarks.find(landmark_id) == vertices_landmarks.end()
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                // connect pose verticies with corresponding landmark verticies 
                if (vertices.contains(frame->keyframe_id) && vertices_landmarks.contains(landmark_id))  //vertices.find(frame->keyframe_id) != vertices.end() && vertices_landmarks.find(landmark_id) != vertices_landmarks.end()
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id));    // pose
                    edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Eigen::Matrix2d::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index++;
                }
                else 
                {
                    delete edge; 
                }
                    
            }
        }

        //create an edge between two pose vertexes and connect thowse two keyframes that were concluded as the same keyframe
        //vertices[loop_candidate_1_id]->setEstimate(keyframes[loop_candidate_2_id]->getPose()); 
        // vertices[loop_candidate_1_id]->setFixed(true);

        // g2o::EdgeSE3 *loop_edge = new g2o::EdgeSE3(); 
        // loop_edge->setId(index); 
        // index++; 
        // loop_edge->setVertex(0, vertices[loop_candidate_1_id]); 
        // loop_edge->setVertex(1, vertices[loop_candidate_2_id]); 
        // loop_edge->setMeasurement(g2o::Isometry3::Identity()); 
        // optimizer.addEdge(loop_edge); 

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }

        std::cout << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                << cnt_inlier << "\n";

        // Set pose and lanrmark position
        for (auto &v : vertices) {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }

}