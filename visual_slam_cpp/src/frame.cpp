#include "myslam/frame.hpp"

namespace myslam 
{

    Frame::Frame(unsigned int in_id, const Sophus::SE3d &in_pose, const cv::Mat &left, const cv::Mat &right)
            : id(in_id), pose_(in_pose), left_img_(left), right_img_(right) {}

    std::shared_ptr<Frame> Frame::CreateFrame() 
    {
        static unsigned int factory_id = 0;
        std::shared_ptr<Frame> new_frame(new Frame);
        new_frame->id = factory_id++;
        return new_frame;
    }

    void Frame::SetKeyFrame() 
    {
        static unsigned int keyframe_factory_id = 0;
        keyframe = true;
        keyframe_id = keyframe_factory_id++;
    }

}
