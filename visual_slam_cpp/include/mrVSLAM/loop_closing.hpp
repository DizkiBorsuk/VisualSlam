/**
 * @file loop_closing.hpp
 * @author mrostocki 
 * @brief loop closer module
 * @version 0.1
 * @date 2024-03-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 
#include "DBoW3/DBoW3.h"

namespace mrVSLAM
{
    class Camera; 
    class Map; 
    class Frame; 
    class KeyFrame; 
    class LocalMapping; 

    class LoopCloser
    {
    public: 
        /**
         * @brief Construct a new Loop Closer object
         * @param vocab_path - path to Bag of words vocabulary 
         */
        LoopCloser(std::string vocab_path)
        {
            DBoW3::Vocabulary vocab(vocab_path); 
            this->bow_database = DBoW3::Database(vocab); 
            this->matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 

            this->loop_closer_running.store(true); 
            this->loop_closer_thread = std::thread(std::bind(&LoopCloser::runLoopCloserThread,this)); 
        } 

        /**
         * @brief Set the Loop Closer object
         * @param map_ptr - pointer to map object 
         * @param local_mapping_ptr - pointer to local mapping object  
         * @param left_cam - pointer to left camera object 
         * @param right_cam - pointer to right camera object 
         */
        void setLoopCloser(std::shared_ptr<Map> map_ptr, std::shared_ptr<LocalMapping> local_mapping_ptr, 
                           std::shared_ptr<Camera> left_cam, std::shared_ptr<Camera> right_cam)
        {
            map = map_ptr; 
            local_mapping = local_mapping_ptr; 
            camera_left = left_cam; 
        }

        /**
         * @brief ends execution of loop closure thread
         */
        void stop(); 

        /**
         * @brief add new keyframe to loop closer module 
         * @param kf ptr to keyframe object  
         */
        void insertKeyframe(std::shared_ptr<KeyFrame> kf); 

    private: 

        void runLoopCloserThread(); 
        /**
         * @brief match features of keyframes that were recognised as a loop closing 
         * @param loop_candidate_kf - ptr to keyframe that is similar to current keyframe
         * @return true 
         * @return false 
         */
        bool matchKeyframes(std::shared_ptr<Frame> loop_candidate_kf); 

    private:
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<LocalMapping> local_mapping = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr, camera_left = nullptr; 

        std::shared_ptr<KeyFrame> current_keyframe = nullptr;
        std::shared_ptr<KeyFrame> last_closed_keyframe = nullptr;  
        std::shared_ptr<KeyFrame> loop_keyframe = nullptr; // keyframe that is connected to current kf 
        std::shared_ptr<KeyFrame> last_keyframe = nullptr; 

        std::mutex loop_closer_mutex; 
        std::thread loop_closer_thread; 
        std::atomic<bool> loop_closer_running; 
        std::condition_variable map_updated; ///< informs loop closer module that there is new keyframe to be processed  

        DBoW3::Database bow_database; ///< database of keyframes or rather their BoW descriptions 

        cv::Ptr<cv::DescriptorMatcher> matcher; // matcher to match two keyframes features 

        std::set<std::pair<int, int>> valid_matches; 

    }; 
} //! end of namespace 