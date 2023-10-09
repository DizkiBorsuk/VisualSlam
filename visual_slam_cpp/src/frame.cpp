#include "../include/frame.hpp"

namespace mrVSLAM
{
    Eigen::Matrix4d Frame::getFramePose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex); 
        return framePose.matrix(); 
    }

    Sophus::SE3d Frame::getSophusFramePose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex); 
        return framePose; 
    }

    // void Frame::SetFramePose(const Eigen::Matrix4d &pose)
    // {
    //     std::lock_guard<std::mutex> lock(pose_mutex);
    //     framePose = pose.matrix();  
    // }

    void Frame::SetFramePose(const Sophus::SE3d &pose)
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