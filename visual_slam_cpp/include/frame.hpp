#pragma once 
#include "common_includes.hpp"

/*
 Frame and Feature objects. 
 Frame is representation of observed scene (by camera) at time t_k. It contains all observed features, 
 camera pose which is rigid body transformation matrix at time t_k, frame id and info if frame is keyframe. 

 Feature is representation of observed features/landmarks at some frame, it's mostly use to abstract and clean Frame class.
 Feature contains position of feature point on 2D image plane 

*/

namespace mrVSLAM
{
    class Frame; 
    class MapPoint; //? https://stackoverflow.com/questions/32014093/shared-ptr-to-abstract-base-class-member-variable-is-a-an-undeclared-identifie

    class Feature
    {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // look in the notes
    public: 
        cv::KeyPoint featurePoint_position; 
        cv::Mat descriptor; // 

        std::weak_ptr<Frame> frame; // frame in which feature was observed, it's weak_ptr because ownership of frame belongs to Map and feature can't own frame (it would create a owhnership loop)
        std::weak_ptr<MapPoint> map_point; // point in map that coresponds to said feature, same as with frame //https://en.cppreference.com/w/cpp/memory/weak_ptr 

        Feature(std::shared_ptr<Frame> in_frame, const cv::KeyPoint &keypoint) noexcept 
            :  featurePoint_position(keypoint), frame(in_frame)
        {   }
        Feature(std::shared_ptr<Frame> in_frame, const cv::KeyPoint &keypoint, const cv::Mat &in_descriptor) noexcept 
            : featurePoint_position(keypoint), descriptor(in_descriptor), frame(in_frame)
        {   }
    }; 

    class Frame
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //### data members ###//
        unsigned int id = 0; // id of a frame 
        unsigned int keyframe_id = 0; // id of a frame if it's a keyframe 
        static unsigned int keyframe_counter; 
        bool is_keyframe = false; // setting if frame is a keyframe 
        //cv::Matx44d framePose = cv::Matx44d::eye(); // pose of a frame 
        Eigen::Matrix4d framePose; //Sophus::SE3d framePose; 

        cv::Mat imgLeft = cv::Mat(370, 1226, CV_8UC1); //? not sure if that is the best way of doing it
        cv::Mat imgRight = cv::Mat(370, 1226, CV_8UC1); 
        
        std::vector<std::shared_ptr<Feature>> features; // features in img/frame (monoSLAM)
        std::vector<std::shared_ptr<Feature>> featuresFromLeftImg; // features in img/frame (stereoSLAM)
        std::vector<std::shared_ptr<Feature>> featuresFromRightImg;

        std::mutex pose_mutex; // mutex for pose to get and set it thread safetly 

        //*### member functions ####// 

        Frame(unsigned int id_in, const Eigen::Matrix4d &pose, const cv::Mat &img) noexcept  // Frame for monocular case 
        : id(id_in), framePose(pose), imgLeft(img)
        {   }
        Frame(unsigned int id_in, const Eigen::Matrix4d &pose, const cv::Mat &img_left, const cv::Mat &img_right) noexcept // Frame for stereo case 
        : id(id_in), framePose(pose), imgLeft(img_left), imgRight(img_right) 
        {   }

        Eigen::Matrix4d getFramePose(); //Sophus::SE3d getFramePose();  // get frame pose from thread 
        Sophus::SE3d getSophusFramePose();
        void SetFramePose(const Eigen::Matrix4d &pose); //void SetFramePose(const Sophus::SE3d &pose);  // set frame pose in thread 
        void SetFrameToKeyframe(); 

    private: 

    }; 
}


