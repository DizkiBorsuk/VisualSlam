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
#include "mrVSLAM/graph_structures.hpp"
#include "DBoW3/DBoW3.h"

namespace mrVSLAM
{
    class Camera; 
    class Map; 
    class Frame; 
    class LocalMapping; 
    
    /**
     * @brief loop closer class/module 
     */
    class LoopCloser
    {
    public: 
        /**
         * @brief Construct a new Loop Closer object
         * @param vocab_path - path to Bag of words vocabulary 
         */
        LoopCloser(std::string vocab_path, bool create_new_descriptors); 
        
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
        void insertKeyframe(std::shared_ptr<Frame> kf); 

    private: 

        void runLoopCloserThread(); 
        /**
         * @brief match features of keyframes that were recognised as a loop closing 
         * @param loop_candidate_kf - ptr to keyframe that is similar to current keyframe
         * @return true 
         * @return false 
         */
        bool matchKeyframesAndCorrectPose(); 

        /**
         * @brief 
         * 
         */
        int optimizePose(Sophus::SE3d& corrected_pose, std::set<std::pair<int, int>> valid_frame_matches); 

        void optimizeLoop(); 

    private:
        bool create_descriptors = false; 
        int loop_closing_counter = 0; 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<LocalMapping> local_mapping = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr, camera_left = nullptr; 

        std::shared_ptr<Frame> current_keyframe = nullptr; ///< currently processed keyframe 
        std::shared_ptr<Frame> loop_keyframe_candidate = nullptr; ///<  

        std::shared_ptr<Frame> prev_keyframe = nullptr; 
        std::shared_ptr<Frame> last_corected_keyframe = nullptr;  


        std::mutex loop_closer_mutex; 
        std::thread loop_closer_thread; 
        std::atomic<bool> loop_closer_running; 
        std::condition_variable new_kf_update; ///< informs loop closer module that there is new keyframe to be processed  

        DBoW3::Database bow_database; ///< database of keyframes or rather their BoW descriptions 
        DBoW3::Vocabulary vocabulary; 
        
        cv::Ptr<cv::DescriptorMatcher> matcher; ///< matcher to match two keyframes features 
        cv::Ptr<cv::DescriptorExtractor> sift_detector; 

        std::set<std::pair<int, int>> valid_matches; 

    }; 
} //! end of namespace 