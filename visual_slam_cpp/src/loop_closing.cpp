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
        loop_closer_thread.join(); 
        fmt::print(fg(fmt::color::indian_red), "loop closer thread closed \n"); 
    }

    void LoopCloser::insertKeyframe(std::shared_ptr<Frame> kf)
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
            cv::Mat descriptors; 
            for(auto &feature : current_keyframe->features_on_left_img)
            {
                descriptors.push_back(feature->descriptor); 
            }
            
            DBoW3::BowVector temp_vector; 
            vocabulary.transform(descriptors, temp_vector); 
            current_keyframe->setBoW_Vector(temp_vector); 
            
            bow_database.add(temp_vector); 
            


            if(bow_database.size() > 20)
            {

            }


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