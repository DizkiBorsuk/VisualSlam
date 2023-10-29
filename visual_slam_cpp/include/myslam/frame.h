#pragma once
#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

// forward declarations
    class MapPoint;
    class Frame; 

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        std::weak_ptr<Frame> frame_; 
        cv::KeyPoint position_;     
        cv::Mat descriptor;         
        std::weak_ptr<MapPoint> map_point_;

        bool is_outlier_ = false;      
        bool is_on_left_image_ = true; 

    public:
        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            : frame_(frame), position_(kp) {}
        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp, cv::Mat in_descriptor)
            : frame_(frame), position_(kp), descriptor(in_descriptor) {}
    };

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        unsigned int id = 0;          
        unsigned int keyframe_id = 0;  
        bool keyframe = false;                     
        Sophus::SE3d pose_;                       
        std::mutex frame_mutex;           
        cv::Mat left_img_, right_img_;   

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:  // data members
        Frame() {}
        Frame(unsigned int in_id, const Sophus::SE3d &in_pose, const cv::Mat &left, const cv::Mat &right);

        // set and get pose, thread safe
        Sophus::SE3d Pose() {
            std::unique_lock<std::mutex> lock(frame_mutex);
            return pose_;
        }

        void SetPose(const Sophus::SE3d &pose) {
            std::unique_lock<std::mutex> lock(frame_mutex);
            pose_ = pose;
        }

        void SetKeyFrame();
        static std::shared_ptr<Frame> CreateFrame();
    };

}  

