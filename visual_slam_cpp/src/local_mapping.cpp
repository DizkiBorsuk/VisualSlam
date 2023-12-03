#include "myslam/local_mapping.hpp"
#include "myslam/g2o_types.hpp"
#include "myslam/mappoint.hpp"

namespace myslam {

LocalMapping::LocalMapping() {
    local_mapping_running.store(true);
    local_mapping_thread = std::thread(std::bind(&LocalMapping::LocalMappingThread, this));
}

void LocalMapping::UpdateMap() {
    std::unique_lock<std::mutex> lock(local_mapping_mutex);
    map_update_.notify_one();
}

void LocalMapping::Stop() {
    local_mapping_running.store(false);
    map_update_.notify_one(); //? 
    local_mapping_thread.join();
}

void LocalMapping::LocalMappingThread() {
    while (local_mapping_running.load()) 
    {
        std::unique_lock<std::mutex> lock(local_mapping_mutex);
        map_update_.wait(lock);

        Map::KeyframesType active_kfs = map->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map->GetActiveMapPoints();
        LocalBundleAdjustment(active_kfs, active_landmarks);
    }
}

void LocalMapping::LocalBundleAdjustment(Map::KeyframesType &keyframes,Map::LandmarksType &landmarks) {
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
        vertex_pose->setId(kf->keyframe_id);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);

        if (kf->keyframe_id > max_kf_id) 
        {
            max_kf_id = kf->keyframe_id;
        }

        vertices.insert({kf->keyframe_id, vertex_pose});
    }

    std::map<unsigned long, VertexXYZ *> vertices_landmarks;
    Eigen::Matrix3d K = cam_left_->getK();
    Sophus::SE3d left_ext = cam_left_->pose();
    Sophus::SE3d right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  
    //double chi2_th = 7.815; 
    std::map<EdgeProjection *, std::shared_ptr<Feature>> edges_and_features;

    for (auto &landmark : landmarks) {
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

}  // namespace myslam