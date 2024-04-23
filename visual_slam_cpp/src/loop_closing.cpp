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
    
    /**
     * @brief 
     * 
     */
    void LoopCloser::runLoopCloserThread()
    {
        while(loop_closer_running.load())
        {

            
            std::unique_lock<std::mutex> lock(loop_closer_mutex);
            new_kf_update.wait(lock); 

            cv::Mat descriptors; 
            for(auto &feature : current_keyframe->features_on_left_img)
            {
                descriptors.push_back(feature->descriptor); 
            }
            
            DBoW3::BowVector current_bow_vector; 
            vocabulary.transform(descriptors, current_bow_vector); 

            current_keyframe->setBoW_Vector(current_bow_vector); 
            bow_database.add(current_bow_vector); 
            
            std::vector<unsigned int> loop_candidate_ids;  
            DBoW3::QueryResults similarity; 

            if(bow_database.size() > 20)
            {
                bow_database.query(current_bow_vector, similarity, 7); 

                for (size_t s = 0; s < similarity.size(); s++)
                {
                    if(similarity.at(s).Score > 0.05 && similarity.at(s).Score < 0.95) {
                        fmt::print(fg(fmt::color::blue), "similarity at index {} for kf {} = {} \n", s, current_keyframe->kf_id, similarity.at(s).Score); 
                        fmt::print(fg(fmt::color::blue), "kf with id {} set as loop candidate for current kf, id = {} \n", similarity.at(s).Id, current_keyframe->kf_id); 
                        loop_candidate_ids.emplace_back(similarity.at(s).Id); 
                    }
                }
            }
            if(loop_candidate_ids.size() == 0) {
                fmt::print(fg(fmt::color::green_yellow), "no loop candidates \n"); 
                continue; 
            }

            if(loop_candidate_ids.size() > 3) {
                fmt::print(fg(fmt::color::green_yellow), "(too) many loop candidates \n"); 
                // continue;
            }
            
            int best_loop_candidate_id = loop_candidate_ids.at(0); 
            int currnet_kf_id = current_keyframe->kf_id; 

            if(currnet_kf_id - best_loop_candidate_id < 20) {
                fmt::print(fg(fmt::color::green_yellow), "kf too close to be considered as loop candidate \n"); 
                continue;
            }
            
            // if all checks are passed then match features of both 
            fmt::print(fg(fmt::color::royal_blue), "kf {} is loop close candidate \n", best_loop_candidate_id); 
            loop_keyframe_candidate = map->getKyeframeById(best_loop_candidate_id); 
            
            matchKeyframes(loop_keyframe_candidate); 


        }
    }

    bool LoopCloser::matchKeyframes(std::shared_ptr<Frame> loop_candidate_kf)
    {
        std::vector<cv::DMatch> matches; 
        cv::Mat descriptors_current, descriptors_loop_candidate; 

        // get features from both keyframes 
        for(size_t i=0; i < current_keyframe->features_on_left_img.size(); i++) {
            descriptors_current.push_back(current_keyframe->features_on_left_img.at(i)->descriptor); 
        }

        for(size_t i=0; i < loop_candidate_kf->features_on_left_img.size(); i++) {
            descriptors_loop_candidate.push_back(loop_candidate_kf->features_on_left_img.at(i)->descriptor); 
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

        fmt::print(fg(fmt::color::red), "loop  closer : number of valid features is = {} \n", found_matches); 

        // ----- END of Matching Part ----- // 

        // ----- Start of Pose correction pose  ----- // 
        std::vector<cv::DMatch> matches_with_mappoints;
        std::vector<cv::Point2f> current_2d_points; 
        std::vector<cv::Point2f> loop_candidate_2d_points;
        std::vector<cv::Point3f> loop_candidate_3d_points;

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

        fmt::print("number of points after map checking ", loop_candidate_3d_points.size()); 
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
            fmt::print("smth wit ransac"); 
        }
        //convert rotation vector to matrix 
        cv::Rodrigues(rotation_vec, R); 

        Eigen::Matrix3d 
        
        return true; 
    }

} //! end of namespace