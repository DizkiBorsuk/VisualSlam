#include "myslam/loop_closing.hpp"
#include "myslam/frame.hpp"
#include "myslam/g2o_types.hpp"
#include "myslam/map.hpp"
#include "myslam/camera.hpp"
#include "myslam/tools.hpp"

namespace myslam
{
    LoopClosing::LoopClosing(std::shared_ptr<DBoW3::Vocabulary> vocab)
    {
        database = DBoW3::Database(*vocab, false, 0); 
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

            database.add(current_frame->bow_vector); 
            DBoW3::QueryResults similarity; 

            unsigned int loop_candidate_id = 0; 

            if(database.size()>10)
            {
                database.query(current_frame->bow_vector, similarity, 0); 

                for(std::size_t s = 0; s < similarity.size(); s++)
                {
                    std::cout << "miara podobieństwa = " <<  similarity.at(s) << "\n";  
                }

                // i only have to check score at second entry because scores are sorted and first one is current frame itself
                // if(similarity.at(1).Score > 0.05 ) 
                // {
                //     std::cout << "for keyframe"  << current_frame->keyframe_id << "found loop candidate with id = " << similarity.at(1).Id << "\n"; 
                //     loop_candidate_id = similarity.at(1).Id; // id of frame that current frame is similar to 
                //     std::cout << " score is " << similarity.at(1).Score << "\n"; 
                //     globalBundleAdjustment(current_frame->keyframe_id, loop_candidate_id); 
                // }
                    
                std::cout << "database size " << database.size() << "\n"; 
            }
        }
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
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);

            if (kf->keyframe_id > max_kf_id) 
            {
                max_kf_id = kf->keyframe_id;
            }

            vertices.insert({kf->keyframe_id, vertex_pose});
        }

        //create an edge between two pose vertexes and connect thowse two keyframes that were concluded as the same keyframe
        


        // map of verticies that represents all map points 
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        Eigen::Matrix3d K = camera_left->getK();
        Sophus::SE3d left_ext = camera_left->pose();
        Sophus::SE3d right_ext = camera_right->pose();

        int index = 1;
        double chi2_th = 5.991;  
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