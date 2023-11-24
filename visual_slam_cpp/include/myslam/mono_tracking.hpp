#pragma once 
#include "myslam/common_include.hpp"
#include "opencv4/opencv2/dnn.hpp"
#include "opencv4/opencv2/dnn/all_layers.hpp"

#include "myslam/frame.hpp"
#include "myslam/map.hpp"

namespace myslam {

    class LocalMapping;
    class LoopClosing; 
    class Visualizer;

    enum class TrackingStatus { INITING, TRACKING, LOST };
    enum class TrackingType {GFTT, ORB, FAST_ORB, SIFT};

    class MonoTracking
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MonoTacking(TrackingType choose_tracking_type, bool destriptors);

        bool AddFrame(std::shared_ptr<Frame> frame);
        void setTracking(std::shared_ptr<Map> map_ptr, std::shared_ptr<LocalMapping> l_mappping_ptr, std::shared_ptr<LoopClosing> lpc_ptr, 
                         std::shared_ptr<Visualizer> viewer_ptr, std::shared_ptr<Camera> cam_l_ptr, std::shared_ptr<DBoW3::Vocabulary> vocab_ptr)
        {
            map = map_ptr; 
            local_mapping = l_mappping_ptr; 
            visualizer = viewer_ptr;
            camera_left = cam_l_ptr;
            loop_closer = lpc_ptr; 
            vocabulary = vocab_ptr; 
        }

    private:
        bool Track();

        int TrackLastFrame();
        int EstimateCurrentPose();
        bool InsertKeyframe();

        int DetectFeatures();
        int extractFeatures(); // extract features from only left img

        void estimateDepth(std::string model_path); 

        bool BuildInitMap();
        int TriangulateNewPoints();

        bool Reset();

        // data
        TrackingStatus status = TrackingStatus::INITING;
        TrackingType type = TrackingType::GFTT; 

        std::shared_ptr<Frame> current_frame = nullptr;  
        std::shared_ptr<Frame> last_frame = nullptr;    
        std::shared_ptr<Camera> camera_left = nullptr;  

        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<Visualizer> visualizer = nullptr;
        std::shared_ptr<LoopClosing> loop_closer = nullptr;
        std::shared_ptr<DBoW3::Vocabulary> vocabulary = nullptr; 

        Sophus::SE3d relative_motion_; 

        int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

        cv::Ptr<cv::FeatureDetector> detector;  // feature detector in opencv
        cv::Ptr<cv::DescriptorExtractor>  extractor; 
        bool use_descriptors = false; 

        static constexpr int GRID_SIZE_H = 46;
        static constexpr int GRID_SIZE_W = 68;


    public:
        // params
        static constexpr int num_features = 150; //150
        static constexpr int num_features_init = 50; // 50 
        static constexpr int num_features_tracking_bad_ = 20; 
        static constexpr int num_features_needed_for_keyframe = 80; //80
    };

}  // namespace myslam

