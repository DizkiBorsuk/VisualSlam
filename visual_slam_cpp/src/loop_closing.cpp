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
                bow_database.query(current_bow_vector, similarity, 10); 

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
                fmt::print(fg(fmt::color::green_yellow), "too many loop candidates \n"); 
                continue;
            }
            
            int best_loop_candidate_id = loop_candidate_ids.at(0); 
            int currnet_kf_id = current_keyframe->kf_id; 

            if(currnet_kf_id - best_loop_candidate_id < 20) {
                fmt::print(fg(fmt::color::green_yellow), "kf too close to be considered as loop candidate \n"); 
                continue;
            }
            
            // if all checks are passed then match features of both 
            loop_keyframe_candidate = map->getKyeframeById(best_loop_candidate_id); 
            // matchKeyframes(loop_keyframe_candidate); 


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

        for(auto &match : matches)
        {

        }


        if(valid_matches.size() < 10)
            return false; 

        return true; 
    }

} //! end of namespace