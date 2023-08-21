#include "../include/frame.hpp"

namespace mrVSLAM
{
    Frame::Frame(unsigned int id, const cv::Matx44d &pose, const cv::Mat &img_left, const cv::Mat &img_right) noexcept
    : id(id), framePose(pose)
    {}

    cv::Matx44d Frame::getFramePose()
    {
        std::lock_guard<std::mutex> lock(poseMutex); 
        return framePose; 
    }

    void Frame::SetFramePose(const cv::Matx44d &pose)
    {
        std::lock_guard<std::mutex> lock(poseMutex);
        framePose = pose;  
    }

    void Frame::SetFrameToKeyFrame()
    {
        is_keyframe = true; 
        keyframe_id = tem_keyframe_id++; 
    }
    

}