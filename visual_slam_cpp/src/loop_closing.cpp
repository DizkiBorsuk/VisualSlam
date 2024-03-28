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

namespace mrVSLAM
{
    void LoopCloser::stop()
    {
        loop_closer_running.store(false); 
        
        loop_closer_thread.join(); 
    }

    void LoopCloser::insertKeyframe(std::shared_ptr<KeyFrame> kf)
    {
        std::unique_lock<std::mutex> lock(loop_closer_mutex); 
        current_keyframe = kf; 

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




            bow_database.add(current_keyframe->bow_vector); 


        }
    }

    bool LoopCloser::matchKeyframes(std::shared_ptr<Frame> loop_candidate_kf)
    {
        std::vector<cv::DMatch> matches; 
        cv::Mat descriptors_current, descriptors_loop_candidate; 

        for(size_t i=0; i < current_keyframe->features_on_left_img.size(); i++)
        {
            descriptors_current.push_back(current_keyframe->features_on_left_img.at(i)->descriptor); 
        }

        for(size_t i=0; i < loop_candidate_kf->features_on_left_img.size(); i++)
        {
            descriptors_loop_candidate.push_back(loop_candidate_kf->features_on_left_img.at(i)->descriptor); 
        }

        matcher->match(descriptors_current, descriptors_loop_candidate, matches); 

        /*
        
        !!! add stuff 
        
        */

        if(valid_matches.size() < 10)
            return false; 

        return true; 
    }

} //! end of namespace