/**
 * @file mono_tracking.hpp
 * @author mrostocki 
 * @brief 
 * @version 0.1
 * @date 2024-03-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 

namespace mrVSLAM
{
    class Camera; 
    class Visualizer; 
    class Map; 
    class Frame; 
    class LoopCloser; 
    class LocalMapping;
    
    //
    class MonoTracking
    {
    public: 
        /**
         * @brief Construct a new Mono Tracking object
         * 
         * @param detector_type - which detector type is used in tracking 
         * @param use_descriptors - if false only keypoint detection will be computed, no descriptors and no loop closure 
         * @param num_features - number of features to detect/extract 
         */
        MonoTracking(const DetectorType& detector_type, const bool& use_descriptors,
                    const unsigned int& num_features); 
        
        /**
         * @brief Set the Tracking object
         * 
         * @param map_ptr - ptr to map object 
         * @param l_mapping_ptr - ptr to local mapping object 
         * @param lpc_ptr - ptr to loop closer object 
         * @param viewer_ptr - ptr to viewer object 
         * @param cam_l_ptr - ptr to left camera object 
         */
        void setTracking(std::shared_ptr<Map> map_ptr, std::shared_ptr<LocalMapping> l_mapping_ptr, std::shared_ptr<LoopCloser> lpc_ptr, 
                         std::shared_ptr<Visualizer> viewer_ptr, std::shared_ptr<Camera> cam_l_ptr)
        {
            map = map_ptr; 
            local_mapping = l_mapping_ptr; 
            loop_closer = lpc_ptr; 
            visualizer = viewer_ptr;
            camera_left = cam_l_ptr;
        }
        
        /**
         * @brief dds new frame object to mono tracking 
         * 
         * @param frame 
         * @return true 
         * @return false 
         */
        bool addNewFrame(std::shared_ptr<Frame> frame); 

    private: 

        void insertKeyframe(); 
        bool monoInitialization(); 
        bool createInitialMap(); 

        bool track(); 

        int detectFeatures(); 
        int extractFeatures(); 

    private: 
        bool use_descriptors = false; 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<LocalMapping> local_mapping = nullptr; 
        std::shared_ptr<Camera> camera_left = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<LoopCloser> loop_closer = nullptr; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr; 

        std::shared_ptr<Frame> reference_kf = nullptr; 
        std::shared_ptr<Frame> current_kf = nullptr; 
        std::shared_ptr<Frame> prev_kf = nullptr; 

        cv::Ptr<cv::FeatureDetector> detector;  // feature detector in opencv
        cv::Ptr<cv::DescriptorExtractor>  extractor; 

        TrackingStatus tracking_status = TrackingStatus::INITING; 
        DetectorType detector_type = DetectorType::GFTT; 

        unsigned int tracking_inliers = 0; 
        unsigned int initialization_counter = 0; 

        unsigned int num_features = 300; 
        static constexpr int num_features_init = 50; 
        static constexpr int num_features_tracking_bad_ = 20; 
        static constexpr int num_features_needed_for_keyframe = 80;

    }; 
} //! end of namespace 