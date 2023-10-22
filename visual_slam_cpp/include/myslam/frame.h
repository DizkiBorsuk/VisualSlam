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
};

class Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    unsigned long id_ = 0;          
    unsigned long keyframe_id_ = 0;  
    bool is_keyframe_ = false;       
    double time_stamp_;              
    Sophus::SE3d pose_;                       
    std::mutex pose_mutex_;           
    cv::Mat left_img_, right_img_;   

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

   public:  // data members
    Frame() {}
    Frame(long id, double time_stamp, const Sophus::SE3d &pose, const cv::Mat &left, const cv::Mat &right);

    // set and get pose, thread safe
    Sophus::SE3d Pose() {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        return pose_;
    }

    void SetPose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        pose_ = pose;
    }

    void SetKeyFrame();
    static std::shared_ptr<Frame> CreateFrame();
};

}  

