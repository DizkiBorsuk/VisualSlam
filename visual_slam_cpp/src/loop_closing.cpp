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


namespace mrVSLAM
{
    LoopCloser::LoopCloser(std::string vocab_path)
    {
        this->vocabulary = DBoW3::Vocabulary(vocab_path); 
        this->bow_database = DBoW3::Database(vocabulary); 
        this->matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 
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

            if(bow_database.size() > 20)
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
            
            int best_loop_candidate_id = loop_candidate_ids.at(0); 
            int currnet_kf_id = current_keyframe->kf_id; 

            if(currnet_kf_id - best_loop_candidate_id < 30) {
                fmt::print(fg(fmt::color::green_yellow), "kf too close to be considered as loop candidate \n"); 
                continue;
            }
            
            // if all checks are passed then match features of both 
            fmt::print(fg(fmt::color::royal_blue), "kf {} is loop close candidate \n", best_loop_candidate_id); 
            this->loop_keyframe_candidate = map->getKyeframeById(best_loop_candidate_id); 
            
            if(matchKeyframesAndCorrectPose()) {
                // optimizeLoop();
                map->addMatchedKeyframes(loop_keyframe_candidate, current_keyframe); 
            }


        }
    }

    bool LoopCloser::matchKeyframesAndCorrectPose()
    {
        std::vector<cv::DMatch> matches; 
        cv::Mat descriptors_current, descriptors_loop_candidate; 

        // get features from both keyframes 
        for(size_t i=0; i < current_keyframe->features_on_left_img.size(); i++) {
            descriptors_current.push_back(current_keyframe->features_on_left_img.at(i)->descriptor); 
        }

        for(size_t i=0; i < this->loop_keyframe_candidate->features_on_left_img.size(); i++) {
            descriptors_loop_candidate.push_back(this->loop_keyframe_candidate->features_on_left_img.at(i)->descriptor); 
        }

        //match keyframes 
        matcher->match(descriptors_current, descriptors_loop_candidate, matches); 

        double min_dist=10000, max_dist=0;
        for (auto& match : matches )
        {
            double dist = match.distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        unsigned int found_matches = 0; 
        std::set<std::pair<int, int> > valid_mathces_ids; 

        for (std::size_t i = 0; i < matches.size(); i++ )
        {
            if(matches[i].distance <= std::max( 2*min_dist, 30.0 ))
            {
                int featId_loop_candidate = matches[i].queryIdx; 
                int featId_current_kframe = matches[i].trainIdx; 

                if(valid_mathces_ids.contains({featId_current_kframe, featId_loop_candidate})){
                    continue;
                }
                valid_mathces_ids.insert({featId_current_kframe, featId_loop_candidate});              
                found_matches++;                                                        
            }
        }

        if(found_matches < 10) {
            fmt::print(fg(fmt::color::red), "not enough matched features between keyframes \n"); 
            return false; 
        }

        fmt::print(fg(fmt::color::red), "loop closer : number of valid features is = {} \n", found_matches); 

        // ----- END of Matching Part ----- // 

        // ----- Start of Pose correction pose  ----- // 
        std::vector<cv::DMatch> matches_with_mappoints;
        std::vector<cv::Point2f> current_2d_points; 
        std::vector<cv::Point2f> loop_candidate_2d_points;
        std::vector<cv::Point3f> loop_candidate_3d_points;

        fmt::print("size of valid matches set = {} \n", valid_mathces_ids.size()); 

        for(auto it = valid_mathces_ids.begin(); it != valid_mathces_ids.end(); )
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
                it = valid_mathces_ids.erase(it); 
            }
        } 

        fmt::print("loop closer : number of points after map checking = {} \n", loop_candidate_3d_points.size()); 
        if(loop_candidate_3d_points.size() < 10){
            return false; 
        }

        //* solve PnP 

        cv::Mat rotation_vec, translation_vec, R, K;
        cv::Mat dist_coeff; // imgs are rectified so this is empty 
        cv::eigen2cv(camera_left->getK(), K); 

        try{
            cv::solvePnPRansac(loop_candidate_3d_points, current_2d_points, K, dist_coeff, 
                     rotation_vec, translation_vec, false, 100, 5.991, 9.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE); 
        } catch(...){
            fmt::print("smth with ransac"); 
            return false; 
        }
        //convert rotation vector to matrix 
        cv::Rodrigues(rotation_vec, R); 

        Eigen::Matrix3d new_R;
        Eigen::Vector3d new_t;  

        cv::cv2eigen(R, new_R); 
        cv::cv2eigen(translation_vec, new_t);

        Sophus::SE3d new_corrected_pose = Sophus::SE3d(new_R, new_t); 

        // int inliers = optimizePose(new_corrected_pose, valid_mathces_ids); // push corrected pose to further optimization 

        //* connect loop keyframes 
        fmt::print(fg(fmt::color::red), "loop closer : corrected pose, setting everything up"); 
        current_keyframe->loop_kf = loop_keyframe_candidate; 
        current_keyframe->setRelativePoseToLoopKf(new_corrected_pose * loop_keyframe_candidate->getPose().inverse() ); 
        last_corected_keyframe = loop_keyframe_candidate; 

        return true; 
    }

    // int LoopCloser::optimizePose(Sophus::SE3d& corrected_pose, std::set<std::pair<int, int>> valid_frame_matches)
    // {
    //     // setup g2o bullshit
    //     typedef g2o::LinearSolverCSparse<g2o::BlockSolver_6_3 ::PoseMatrixType> LinearSolverType;
    //     auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3 >(std::make_unique<LinearSolverType>()));
    //     g2o::SparseOptimizer optimizer;
    //     optimizer.setAlgorithm(solver);

    //     VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    //     vertex_pose->setId(0);
    //     vertex_pose->setEstimate(corrected_pose);
    //     optimizer.addVertex(vertex_pose);

    //     Eigen::Matrix3d K = camera_left->getK();

    //     int index = 1;
    //     std::vector<EdgeProjectionPoseOnly *> edges;
    //     std::vector<std::shared_ptr<Feature>> features;
    //     std::vector<bool> is_edge_an_outlier; 

    // }

    // void LoopCloser::optimizeLoop()
    // {
    //     local_mapping->requestPause(); 

    //     if(!local_mapping->confirmPause()) {
    //         fmt::print(fg(fmt::color::yellow_green), "waiting for local mapping to pause"); 
    //         std::this_thread::sleep_for(500us); 
    //     }

    //     {
    //         std::unique_lock<std::mutex> lock(map->mapUpdate_mutex); 

    //         std::unordered_map<unsigned int, Sophus::SE3d> correctedPoses; 

    //         correctedPoses.insert({current_keyframe->kf_id, }); 

    //         for(auto &keyframe: map)

    //     }

    //     local_mapping->resume(); 
    // }

} //! end of namespace