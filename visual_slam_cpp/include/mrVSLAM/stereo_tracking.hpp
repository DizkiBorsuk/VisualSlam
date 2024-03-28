/**
 * @file stereo_tracking.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-09
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
    class KeyFrame; 
    class LoopCloser; 
    class LocalMapping; 

    class StereoTracking
    {
    public:
        /**
         * @brief Construct a new Stereo Tracking object
         * 
         * @param detector_type 
         * @param use_descriptors 
         * @param num_features 
         */
        StereoTracking(const DetectorType& detector_type, const bool& use_descriptors,
                       const unsigned int& num_features); 

        /**
         * @brief Set the Tracking object
         * 
         * @param map_ptr 
         * @param l_mapping_ptr 
         * @param lpc_ptr 
         * @param viewer_ptr 
         * @param cam_l_ptr 
         * @param cam_r_ptr 
         */
        void setTracking(std::shared_ptr<Map> map_ptr, std::shared_ptr<LocalMapping> l_mapping_ptr, std::shared_ptr<LoopCloser> lpc_ptr, 
                         std::shared_ptr<Visualizer> viewer_ptr, std::shared_ptr<Camera> cam_l_ptr, std::shared_ptr<Camera> cam_r_ptr )
        {
            map = map_ptr; 
            local_mapping = l_mapping_ptr; 
            visualizer = viewer_ptr;
            camera_left = cam_l_ptr;
            camera_right = cam_r_ptr;
            loop_closer = lpc_ptr; 
        }

        /**
         * @brief adds new frame object to stereo tracking 
         * 
         * @param frame 
         * @return true 
         * @return false 
         */
        bool addNewFrame(std::shared_ptr<Frame> frame); 

    private: 
        bool stereoInitialize(); 
        bool createInitialMap(); 
        int triangulateNewPoints(); 

        bool track(); 
        int trackLastFrame(); 
        void estimateCurrentPose(); 
        void insertKeyframe(); 

        int findCorrespondingPoints(); 
        int detectFeatures(); 
        int extractFeatures(); 

    private: 

        bool use_descriptors = false; 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<LocalMapping> local_mapping = nullptr; 
        std::shared_ptr<Camera> camera_left = nullptr, camera_right = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<LoopCloser> loop_closer = nullptr; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr; 
        std::shared_ptr<KeyFrame> reference_kf = nullptr; 

        cv::Ptr<cv::FeatureDetector> detector;  // feature detector in opencv
        cv::Ptr<cv::DescriptorExtractor>  extractor; 

        TrackingStatus tracking_status = TrackingStatus::INITING; 
        DetectorType detector_type = DetectorType::GFTT; 

        int tracking_inliers = 0; 

        unsigned int num_features = 150; 
        static constexpr int num_features_init = 50; 
        static constexpr int num_features_tracking_bad_ = 20; 
        static constexpr int num_features_needed_for_keyframe = 80;
    }; 

} //! end of namespace 