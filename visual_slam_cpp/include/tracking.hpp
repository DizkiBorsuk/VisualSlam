#pragma once 
#include "common_includes.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "camera.hpp"

/*
    Front-end 
    Tracking is responsible for calculating frame's pose based on img/imgs, feature detection, map initalization 
*/

namespace mrVSLAM
{
    class LocalMapping; 
    class Visualizer; 

    enum class STATUS {INITIALIZATION, TRACKING, LOST}; // status of tracking 
    enum class DETECTOR {GFTT, FAST}; 
    enum class EXTRACTOR {ORB, SIFT, AKAZE}; 

    class Tracking
    {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    public: 
        //### function members ###// 
        Tracking(DETECTOR); 
        Tracking(EXTRACTOR); 
        void setTracking(std::shared_ptr<Map> in_map, std::shared_ptr<Visualizer> in_visualizer, std::shared_ptr<LocalMapping> in_lMapping, 
                        std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right);
        //void addFrameAndTrack(std::shared_ptr<Frame> frame_to_add);
        bool addFrameAndTrackStereo(std::shared_ptr<Frame> frame_to_add); // main function of tracking

        // 3 state functions 
        bool initialize(); // initialization for monocular case 
        bool stereoInitialize(); // initialization for stereo case 
        void track(); 
        void restartTracking(); 

        // 
        void newKeyframeInsertion(); 

        //feature detection functions 
        unsigned int detectFeatures(); // detect features in frame left img and return number of found points
        unsigned int extractFeatures(); // extract keypoints and descriptors
        unsigned int findCorrespondingStereoFeatures(); 
        
        unsigned int estimatePose(); 
        
        // 
        void buildMap(); 
        void createNewMapPoints(); 

    private: 

        bool initSucces = false; 
        STATUS tracking_status = STATUS::INITIALIZATION; 
        
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptorExtractor; 

        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Frame> prev_frame = nullptr;
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<LocalMapping> l_mapping = nullptr; 
        std::shared_ptr<Camera> camera_left = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr; 


        unsigned int num_of_features = 600; 
        unsigned int num_of_features_for_initialization = 100; // numbers of features needed to be found in both imgs for initalization success 
        unsigned int num_of_features_for_keyframe = 50; //? check in orbslam 

        //Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity(); //? maybe change to eigen or sophus 
        Sophus::SE3d transformationMatrix; 

        static constexpr double chi_squared_treshold = 5.991; // {9.210,7.378,5.991,5.991};
    }; 
}