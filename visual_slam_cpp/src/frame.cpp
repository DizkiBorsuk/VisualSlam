#include "../include/frame.hpp"

namespace mrVSLAM
{
    Frame::Frame(unsigned int id, const Eigen::Matrix4d &pose, const cv::Mat &img_left) noexcept
    : id(id), framePose(pose), imgLeft(img_left)
    {   }

    Frame::Frame(unsigned int id, const Eigen::Matrix4d &pose, const cv::Mat &img_left, const cv::Mat &img_right) noexcept
    : id(id), framePose(pose)
    {   }

    Eigen::Matrix4d Frame::getFramePose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex); 
        return framePose; 
    }

    void Frame::SetFramePose(const Eigen::Matrix4d &pose)
    {
        std::lock_guard<std::mutex> lock(pose_mutex);
        framePose = pose;  
    }

    void Frame::SetFrameToKeyframe()
    {
        is_keyframe = true; 
        keyframe_id = keyframe_counter++; 
    }
    

}