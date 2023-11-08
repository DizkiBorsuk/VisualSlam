#pragma once

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

    class LocalMapping;
    class Visualizer;

    enum class TrackingStatus { INITING, TRACKING, LOST };
    enum class TrackingType { OpticalFlow_GFTT, OpticalFlow_ORB, Matching_ORB, Matching_SIFT};

    class StereoTracking 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        StereoTracking(TrackingType choose_tracking_type);

        bool AddFrame(std::shared_ptr<Frame> frame);
        void setTracking(std::shared_ptr<Map> map_ptr, std::shared_ptr<LocalMapping> l_mappping_ptr, 
                        std::shared_ptr<Visualizer> viewer_ptr, std::shared_ptr<Camera> cam_l_ptr, std::shared_ptr<Camera> cam_r_ptr)
        {
            map = map_ptr; 
            local_mapping = l_mappping_ptr; 
            viewer_ = viewer_ptr;
            camera_left_ = cam_l_ptr;
            camera_right_ = cam_r_ptr;
        }

    private:
        bool Track();

        int TrackLastFrame();
        int EstimateCurrentPose();
        bool InsertKeyframe();
        bool StereoInit();

        int DetectFeatures();
        int extractFeatures(); // extract features from only left img
        int extractStereoFeatures(); // extract features from both imgs 

        int findCorrespondensesWithOpticalFlow();
        int findCorrespondensesWithMatching();

        bool BuildInitMap();

        int TriangulateNewPoints();

        bool Reset();

        // data
        TrackingStatus status = TrackingStatus::INITING;
        TrackingType type = TrackingType::OpticalFlow_GFTT; 

        std::shared_ptr<Frame> current_frame = nullptr;  
        std::shared_ptr<Frame> last_frame = nullptr;    
        std::shared_ptr<Camera> camera_left_ = nullptr;  
        std::shared_ptr<Camera> camera_right_ = nullptr;  

        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<Visualizer> viewer_ = nullptr;

        Sophus::SE3d relative_motion_; 

        int tracking_inliers_ = 0;  // inliers, used for testing new keyframes




        cv::Ptr<cv::FeatureDetector> detector;  // feature detector in opencv
        cv::Ptr<cv::DescriptorExtractor>  extractor; 
        cv::Ptr<cv::DescriptorMatcher> matcher;


        static constexpr int grid_rows = 24;  
        static constexpr int grid_cols = 32;  

    public:
                // params
        static constexpr int num_features = 550;
        static constexpr int num_features_init = 50;
        static constexpr int num_features_tracking_bad_ = 20;
        static constexpr int num_features_needed_for_keyframe_ = 200;
    };

}  // namespace myslam

