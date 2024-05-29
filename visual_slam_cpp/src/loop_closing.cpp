/**
 * @file loop_closing.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mrVSLAM/loop_closing.hpp" 
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/map.hpp"
#include "mrVSLAM/mappoint.hpp"
#include "mrVSLAM/camera.hpp"
#include "mrVSLAM/local_mapping.hpp"
#include "mrVSLAM/tools.hpp"


namespace mrVSLAM
{
    LoopCloser::LoopCloser(std::string vocab_path, bool create_new_descriptors)
    {
        this->vocabulary = DBoW3::Vocabulary(vocab_path); 
        this->bow_database = DBoW3::Database(vocabulary); 
        this->create_descriptors = create_new_descriptors; 

        if(create_new_descriptors){
            this->sift_detector = cv::SIFT::create(500, 3, 0.04, 10, 1.6, false); 
        }

        this->matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        this->loop_closer_running.store(true); 

        this->loop_closer_thread = std::thread(std::bind(&LoopCloser::runLoopCloserThread,this)); 
        fmt::print(fg(fmt::color::aqua), "loop closer thread started \n"); 
    } 

    void LoopCloser::stop()
    {
        loop_closer_running.store(false); 
        new_kf_update.notify_one(); 
        loop_closer_thread.join(); 
        fmt::print(fg(fmt::color::indian_red), "loop closer thread closed \n"); 
    }

    void LoopCloser::insertKeyframe(std::shared_ptr<Frame> kf)
    {
        std::unique_lock<std::mutex> lock(loop_closer_mutex); 
        current_keyframe = kf; 
        new_kf_update.notify_one(); 
    }
    
    void LoopCloser::runLoopCloserThread()
    {
        while(loop_closer_running.load())
        {
            std::unique_lock<std::mutex> lock(loop_closer_mutex);
            new_kf_update.wait(lock); 
            
            // get all descriptors for current keyframe as one cv::Mat 
            cv::Mat descriptors; 
            for(auto &feature : current_keyframe->features_on_left_img) {
                descriptors.push_back(feature->descriptor); 
            }

            // create BoW vector for current keyframe and add to database and current frame object 
            DBoW3::BowVector current_bow_vector; 
            vocabulary.transform(descriptors, current_bow_vector); 

            current_keyframe->setBoW_Vector(current_bow_vector); 
            bow_database.add(current_bow_vector); 
            
            // calculate similarity 
            std::vector<unsigned int> loop_candidate_ids;  
            DBoW3::QueryResults similarity; 

            if(bow_database.size() > 30)
            {
                bow_database.query(current_bow_vector, similarity, 7); 

                for (size_t s = 0; s < similarity.size(); s++)
                {
                    if(similarity.at(s).Score > 0.055 && similarity.at(s).Score < 0.95) {
                        fmt::print(fg(fmt::color::blue), "similarity at index {} for kf {} = {} \n", s,similarity.at(s).Id, similarity.at(s).Score); 
                        fmt::print(fg(fmt::color::blue), "kf with id {} set as loop candidate for current kf, id = {} \n", similarity.at(s).Id, current_keyframe->kf_id); 
                        loop_candidate_ids.emplace_back(similarity.at(s).Id); 
                    }
                }
            }
            if(loop_candidate_ids.size() == 0) {
                fmt::print(fg(fmt::color::green_yellow), "no loop candidates \n"); 
                continue; 
            }

            //TODO maybe change it 
            if(loop_candidate_ids.size() > 4) {
                fmt::print(fg(fmt::color::green_yellow), "(too) many loop candidates \n"); 
                continue;
            }

            int best_loop_candidate_id = loop_candidate_ids.at(0); //same as kf since only keyframes are added tp database
            int currnet_kf_id = current_keyframe->kf_id; 

            loop_keyframe_candidate = map->getKyeframeById(best_loop_candidate_id); 

            if(currnet_kf_id - best_loop_candidate_id < 40) {
                fmt::print(fg(fmt::color::green_yellow), "kf too close to be considered as loop candidate \n"); 
                continue;
            }

            // if all checks are passed then match features of both 
            if(loop_keyframe_candidate->used_in_loop_closing) {
                fmt::print(fg(fmt::color::green_yellow), "kf was already used in loop closing \n"); 
                continue; 
            }

            double x1 = loop_keyframe_candidate->getPose().inverse().matrix3x4().coeff(0,3); 
            double y1 = loop_keyframe_candidate->getPose().inverse().matrix3x4().coeff(2,3); 
            double x2 = current_keyframe->getPose().inverse().matrix3x4().coeff(0,3); 
            double y2 = current_keyframe->getPose().inverse().matrix3x4().coeff(2,3); 

            if((std::abs(x1 - x2) > 50)  || (std::abs(y1-y2) > 50)) {
                fmt::print(fg(fmt::color::red), "loop closer : frames to far apart from each other \n"); 
                continue;
            }

            fmt::print(fg(fmt::color::royal_blue), "kf {} is loop close candidate \n", best_loop_candidate_id); 
            
            if(matchKeyframesAndCorrectPose()) {
                optimizeLoop();
                map->addMatchedKeyframes(loop_keyframe_candidate, current_keyframe); 
            }

        }
    }

    bool LoopCloser::matchKeyframesAndCorrectPose()
    {
        std::vector<cv::DMatch> good_matches; 
        std::vector<std::vector<cv::DMatch>> matches; 
        cv::Mat descriptors_current, descriptors_loop_candidate; 

        std::cout << "loop closer : matching \n"; 
        // get features from both keyframes 

        if(create_descriptors){
            std::vector<cv::KeyPoint> keypoints_current, keypoints_candidate; 
            for(auto &feature : current_keyframe->features_on_left_img){
                keypoints_current.emplace_back(feature->positionOnImg); 
            }

            sift_detector->compute(current_keyframe->left_img, keypoints_current, descriptors_current); 

            for(auto &feature : loop_keyframe_candidate->features_on_left_img){
                keypoints_candidate.emplace_back(feature->positionOnImg); 
            }

            sift_detector->compute(loop_keyframe_candidate->left_img, keypoints_candidate, descriptors_loop_candidate); 

        } else {
            for(size_t i=0; i < current_keyframe->features_on_left_img.size(); i++) {
                descriptors_current.push_back(current_keyframe->features_on_left_img.at(i)->descriptor); 
            }

            for(size_t i=0; i < this->loop_keyframe_candidate->features_on_left_img.size(); i++) {
                descriptors_loop_candidate.push_back(this->loop_keyframe_candidate->features_on_left_img.at(i)->descriptor); 
            }

            descriptors_current.convertTo(descriptors_current, CV_32F); 
            descriptors_loop_candidate.convertTo(descriptors_loop_candidate, CV_32F); 
        }

        fmt::print("loop closer : current frame num of descriptors = {}, second frame num of descriptors = {} \n", current_keyframe->features_on_left_img.size(), loop_keyframe_candidate->features_on_left_img.size()); 

        unsigned int found_matches = 0; 
        valid_matches.clear(); 

        //match keyframes 
        // matcher->match(descriptors_current, descriptors_loop_candidate, matches); 
        // matcher->radiusMatch(descriptors_current, descriptors_loop_candidate, matches2, 0.4); 
        matcher->knnMatch(descriptors_current, descriptors_loop_candidate, matches, 2); 

        double ratio = 0.85; 
        std::vector<std::vector<cv::DMatch>>::iterator iter;
        for(iter = matches.begin(); iter != matches.end(); iter++)
        {
            if((*iter)[0].distance / (*iter)[1].distance < ratio ){

                int featId_loop_candidate = (*iter)[0].trainIdx; 
                int featId_current_kframe = (*iter)[0].queryIdx; 
                
                if(valid_matches.contains({featId_current_kframe, featId_loop_candidate})){
                    continue;
                }
                valid_matches.insert({featId_current_kframe, featId_loop_candidate});         
                good_matches.emplace_back((*iter)[0]);      
                found_matches++; 
            }
        }

        fmt::print("loop closer : found {} matches \n", found_matches); 
        /*
        double min_dist=10000, max_dist=0;
        for (auto& match : matches )
        {
            double dist = match.distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        fmt::print("loop closer: min dist = {}, max_dist = {} \n", min_dist, max_dist); 

        for (std::size_t i = 0; i < matches.size(); i++ )
        {
            if(matches[i].distance <= std::max( 2*min_dist, 10.0 ))
            {
                int featId_loop_candidate = matches[i].trainIdx; 
                int featId_current_kframe = matches[i].queryIdx; 

                if(valid_matches.contains({featId_current_kframe, featId_loop_candidate})){
                    continue;
                }
                valid_matches.insert({featId_current_kframe, featId_loop_candidate});              
                found_matches++;                                                        
            }
        }
        */

        if(true){
            //  show the match result
            cv::Mat matches_img;
            cv::drawMatches(current_keyframe->left_img, current_keyframe->getFrameKeypoints(), 
                    loop_keyframe_candidate->left_img, loop_keyframe_candidate->getFrameKeypoints(), 
                    good_matches, matches_img);
            // cv::resize(matches_img, matches_img, cv::Size(), 0.5, 0.5);

            std::string path_to_save = "/home/maciek/dev/projects_cpp/VisualSlam/results/matching_" + std::to_string(loop_closing_counter) + ".jpg"; 
            cv::imwrite(path_to_save, matches_img); 
            loop_closing_counter++;
        }

        if(found_matches < 50) {
            fmt::print(fg(fmt::color::red), "not enough matched features between keyframes \n"); 
            return false; 
        }

        // fmt::print(fg(fmt::color::red), " number of valid features is = {} \n", found_matches); 

        // ----- END of Matching Part ----- // 

        // ----- Start of Pose correction pose  ----- // 
        std::vector<cv::DMatch> matches_with_mappoints;
        std::vector<cv::Point2d> current_2d_points; 
        std::vector<cv::Point2d> loop_candidate_2d_points;
        std::vector<cv::Point3d> loop_candidate_3d_points;

        // fmt::print("loop closer : size of valid matches set = {} \n", valid_matches.size()); 

        for(auto it = valid_matches.begin(); it != valid_matches.end(); )
        {
            int featId_current_kframe = (*it).first; 
            int featId_loop_candidate = (*it).second; 

            auto map_point = loop_keyframe_candidate->features_on_left_img[featId_loop_candidate]->map_point.lock(); 
            if(map_point) 
            {
                current_2d_points.emplace_back(current_keyframe->features_on_left_img[featId_current_kframe]->positionOnImg.pt); 
                
                Eigen::Vector3d position = map_point->getPointPosition(); 
                loop_candidate_3d_points.emplace_back((position(0), position(1), position(2))); 
                loop_candidate_2d_points.emplace_back(loop_keyframe_candidate->features_on_left_img[featId_loop_candidate]->positionOnImg.pt); 

                // cv::DMatch valid_match(featId_current_kframe, featId_loop_candidate, 10); 
                matches_with_mappoints.emplace_back(featId_current_kframe, featId_loop_candidate, 10); 
                it++; 
            } else {
                it = valid_matches.erase(it); 
            }
        } 

        fmt::print("loop closer : number of points after map checking = {} \n", loop_candidate_3d_points.size()); 
        
        if(loop_candidate_3d_points.size() < 30){
            return false; 
        }

        //* solve PnP 
        cv::Mat rotation_vec, translation_vec, R, K;
        cv::eigen2cv(camera_left->getK(), K); 

        try{
            // cv::solvePnPRansac(loop_candidate_3d_points, current_2d_points, K, cv::noArray(), 
            //          rotation_vec, translation_vec, false, 500, 5.991, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP); 
            cv::solvePnPRansac(loop_candidate_3d_points, current_2d_points, K, cv::noArray(), 
                     rotation_vec, translation_vec, false, 200, 5.991, 0.99); 
            //cv::solvePnP(loop_candidate_3d_points, current_2d_points, K, cv::Mat(), rotation_vec, translation_vec, false,cv::SOLVEPNP_ITERATIVE); 
        } catch(...){
            fmt::print("smth with ransac \n"); 
            return false; 
        }
        //convert rotation vector to matrix 
        cv::Rodrigues(rotation_vec, R); 

        Eigen::Matrix3d new_R;
        Eigen::Vector3d new_t;  

        cv::cv2eigen(R, new_R); 
        cv::cv2eigen(translation_vec, new_t);
        
        Sophus::SE3d pnp_pose; 
        try {
            pnp_pose = Sophus::SE3d(new_R, new_t); 
        } catch(...) {
            fmt::print(fg(fmt::color::red), "loop closer : wrong R matrix \n");
            return false;
        }

        std::cout << "pose before correction : \n" << current_keyframe->getPose().matrix3x4() << "\n"; 
        std::cout << "pnp pose : \n" << pnp_pose.matrix3x4() << "\n"; 

        int inliers = optimizePose(pnp_pose); // push corrected pose to further optimization 

        if(inliers < 20) {
            fmt::print(fg(fmt::color::red), "number of inliers: {} is to small \n", inliers); 
            return false; 
        }

        std::cout << "pose after optimization : \n" << corrected_kf_pose.matrix3x4() << "\n"; 

        //* connect loop keyframes 
        fmt::print(fg(fmt::color::red), "loop closer : corrected pose, setting everything up \n"); 

        current_keyframe->loop_kf = loop_keyframe_candidate; 
        loop_keyframe_candidate->loop_kf = current_keyframe; 

        current_keyframe->used_in_loop_closing = true; 
        loop_keyframe_candidate->used_in_loop_closing = true; 

        current_keyframe->setRelativePoseToLoopKf(corrected_kf_pose * loop_keyframe_candidate->getPose().inverse() ); 
        last_corected_keyframe = loop_keyframe_candidate; 

        return true; 
    }

    int LoopCloser::optimizePose(Sophus::SE3d& pnp_pose)
    {
        // setup g2o bullshit
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(pnp_pose);
        optimizer.addVertex(vertex_pose);

        Eigen::Matrix3d K = camera_left->getK();

        //define edges of a graph 
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<std::shared_ptr<Feature>> features;

        std::vector<bool> is_edge_an_outlier; 
        std::vector<std::pair<int, int>> valid_matches_checked; 

        for(auto iter = this->valid_matches.begin(); iter != this->valid_matches.end(); iter++)
        {
            int current_feature_id = (*iter).first;  
            int loop_candidate_feature_id = (*iter).second; 

            auto loop_candidate_mp = this->loop_keyframe_candidate->features_on_left_img.at(loop_candidate_feature_id)->map_point.lock(); 
            auto current_2d_point = this->current_keyframe->features_on_left_img.at(current_feature_id)->positionOnImg.pt; 

            assert(loop_candidate_mp); 
            // if(!loop_candidate_mp) {continue;}

            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(loop_candidate_mp->getPointPosition(), K);
            edge->setId(index); 
            edge->setVertex(0, vertex_pose); 
            edge->setMeasurement(convertToVec(current_2d_point)); 
            edge->setInformation(Eigen::Matrix2d::Identity()); 
            edge->setRobustKernel(new g2o::RobustKernelHuber); 
            
            edges.emplace_back(edge); 
            is_edge_an_outlier.emplace_back(false); 
            valid_matches_checked.emplace_back(*iter); 

            optimizer.addEdge(edge);
            index++; 
        }

        constexpr double chi2 = 5.991; 
        unsigned int outliers = 0; 
        int iterations = 7; 

        for(int iteration = 0; iteration < iterations; iteration++)
        {
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        outliers = 0;

        // count the outliers
            for(size_t i = 0; i < edges.size(); i++)
            {
                auto e = edges[i];
                if(is_edge_an_outlier[i]){
                    e->computeError();
                }
                if(e->chi2() > chi2){
                    is_edge_an_outlier[i] = true;
                    e->setLevel(1);
                    outliers++;
                } else{
                    is_edge_an_outlier[i] = false;
                    e->setLevel(0);
                }

                if(iteration == 2){
                    e->setRobustKernel(nullptr);
                }
            }
        }

        for(size_t i = 0; i < is_edge_an_outlier.size(); i++) {
            if(is_edge_an_outlier[i]) {
                this->valid_matches.erase(valid_matches_checked[i]); 
            }
        }

        this->corrected_kf_pose = vertex_pose->estimate(); 

        unsigned int inliers = this->valid_matches.size(); 
        fmt::print("loop closer : num of inliers = {}, num of outliers = {} \n", inliers, outliers);

        return inliers; 
    }

    void LoopCloser::optimizeLoop()
    {
        //*stop local mapping 
        local_mapping->requestPause(); 

        if(!local_mapping->confirmPause()) {
            fmt::print(fg(fmt::color::yellow_green), "waiting for local mapping to pause"); 
            std::this_thread::sleep_for(500us); 
        }

        //* 
        {
            std::unique_lock<std::mutex> lock(map->mapUpdate_mutex); 

            std::unordered_map<unsigned int, Sophus::SE3d> corrected_active_poses; 
            corrected_active_poses.insert({current_keyframe->kf_id, this->corrected_kf_pose});
            
            auto active_keyframes = map->getActiveKeyframes(); 
            for(auto &keyframe: active_keyframes) {
                
                auto kf_id = keyframe.first; 
                if(kf_id == current_keyframe->kf_id) {continue;}
                
                Sophus::SE3d T = keyframe.second->getPose() * (current_keyframe->getPose().inverse()); // transformation matrix between current keyframa and x keyframe 
                Sophus::SE3d T_corrected = T * corrected_kf_pose; 
                corrected_active_poses.insert({kf_id, T_corrected});
                fmt::print("loop closer : stored kf id = {} \n", kf_id); 
            }

            auto active_mappoints = map->getActiveMappoints(); 
            unsigned int shifted_points = 0; 
            for(auto& mappoint_set : active_mappoints) {
                    auto mappoint = mappoint_set.second;

                    assert(!mappoint->getActiveObservations().empty()); 

                    auto feature = mappoint->getActiveObservations().front().lock(); 
                    auto observed_kf = feature->keyframe.lock(); 

                    if(!corrected_active_poses.contains(observed_kf->kf_id)) {
                        fmt::print("tried to acces kf {}, no such kf in corrected poses",observed_kf->kf_id); 
                        continue;} 

                    Eigen::Vector3d kf_position = observed_kf->getPose() * mappoint->getPointPosition(); 
                    Sophus::SE3d T_corrected = corrected_active_poses.at(observed_kf->kf_id); 
                    mappoint->setPosition(T_corrected.inverse() * kf_position); 
            }

            for(auto& kf : active_keyframes) {
                kf.second->setPose(corrected_active_poses.at(kf.first)); 
            }

            for(auto iter = valid_matches.begin(); iter != valid_matches.end(); iter++){
                int current_feature_id = (*iter).first;
                int loop_candidate_feat_id = (*iter).second;

                auto loop_mp = loop_keyframe_candidate->features_on_left_img[loop_candidate_feat_id]->map_point.lock();

                if(loop_mp){
                    auto current_mp = current_keyframe->features_on_left_img[current_feature_id]->map_point.lock();
                    if(current_mp){
                        // link the current mappoint's observation to the matched loop mappoint
                        for(auto &obs: current_mp->getObservations()){
                            auto obs_feat = obs.lock();
                            loop_mp->addObservation(obs_feat);
                            obs_feat->map_point = loop_mp;
                        }
                        // then, remove the current mappoint from the map
                        map->removeMapPoint(current_mp);
                    }else{
                        current_keyframe->features_on_left_img[current_feature_id]->map_point = loop_mp;
                    }            
                }
            }
        } // end of lock 

        //* start of graph optimization 
        
        // setup g2o
        typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        auto all_keyframes = map->getAllKeyframes(); 
        auto active_kf = map->getActiveKeyframes(); 

        std::map<unsigned int, VertexPose*> kf_verticies; 

        for(auto& kf_map_entry : all_keyframes) 
        {
            unsigned int kf_id = kf_map_entry.first; 
            auto keyframe = kf_map_entry.second; 

            VertexPose *pose_vertex = new VertexPose(); 
            pose_vertex->setId(kf_id); 
            pose_vertex->setEstimate(keyframe->getPose()); 
            pose_vertex->setMarginalized(false); 

            if(active_kf.contains(kf_id) || kf_id == loop_keyframe_candidate->kf_id || kf_id == 0) 
            {
                pose_vertex->setFixed(true); 
            }

            optimizer.addVertex(pose_vertex); 
            kf_verticies.insert({kf_id, pose_vertex}); 
        }

        int index = 0; 
        std::map<unsigned int, EdgePoseGraph*> edges;  
        for(auto& kf_map_entry : all_keyframes)
        {
            unsigned int kf_id = kf_map_entry.first; 
            auto keyframe = kf_map_entry.second; 

            auto prev_kf = keyframe->prev_kf.lock(); 
            assert(prev_kf != nullptr); 

            // create edges between two kf, length of edge is distance between them i.e 
            if(prev_kf) {
                EdgePoseGraph * edge = new EdgePoseGraph(); 
                edge->setId(index); 
                edge->setVertex(0, kf_verticies.at(kf_id)); 
                edge->setVertex(1, kf_verticies.at(prev_kf->kf_id)); 
                edge->setMeasurement(keyframe->getRelativePoseToLastKf()); 
            }         
        }


        //*resume local mapping 
        local_mapping->resume(); 
    }

} //! end of namespace