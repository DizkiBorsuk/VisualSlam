/**
 * @file frame.cpp
 * @author mrostocki 
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mrVSLAM/frame.hpp"

namespace mrVSLAM
{
    /**
     * @brief Construct a new Frame:: Frame object
     * 
     * @param frame_id 
     * @param leftImg 
     * @param rightImg 
     */
    Frame::Frame(const unsigned int frame_id, const cv::Mat &leftImg, const cv::Mat &rightImg)
    {
        id = frame_id; 
        left_img = leftImg; 
        right_img = rightImg; 
    }
    /*
        Frame setters getters 
    */
    Sophus::SE3d Frame::getPose()
    {
        std::unique_lock<std::mutex> lock(frame_mutex); 
        return pose; 
    }

    void Frame::setPose(const Sophus::SE3d &in_pose)
    {
        std::unique_lock<std::mutex> lock(frame_mutex); 
        pose = in_pose; 
    }

//* ------------------------ *//

    void Frame::setFrameToKeyframe()
    {
        static unsigned int factory_kf_id = 0; 
        this->kf_id = factory_kf_id; 
        factory_kf_id++; 
        fmt::print(bg(fmt::color::gray), "frame id: {}, set to keyframe id: {} \n", this->id, this->kf_id); 
    }

    void Frame::setRelativePoseToLastKf(const Sophus::SE3d &in_pose)
    {
        std::unique_lock<std::mutex> lock(frame_mutex); 
        this->relativePoseToLastKf = in_pose; 
    }

    void Frame::setRelativePoseToLoopKf(const Sophus::SE3d &in_pose)
    {
        std::unique_lock<std::mutex> lock(frame_mutex); 
        this->relativePoseToLoopKf = in_pose; 
    }

    void Frame::setBoW_Vector(DBoW3::BowVector &bow_vec)
    {
        std::unique_lock<std::mutex> lock(frame_mutex); 
        this->bow_vector = bow_vec; 
    }

    Sophus::SE3d Frame::getRelativePoseToLastKf()
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        return relativePoseToLastKf; 
    }

    Sophus::SE3d Frame::getRelativePoseToLoopKf()
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        return relativePoseToLoopKf; 
    }

    DBoW3::BowVector Frame::getBoW_Vector()
    {
        std::unique_lock<std::mutex> lock(frame_mutex);
        return bow_vector; 
    }


    // /**
    //  * @brief Construct a new Key Frame:: Key Frame object
    //  * @details
    //  * @param base_frame 
    //  */
    // KeyFrame::KeyFrame(std::shared_ptr<Frame> base_frame)
    // {
    //     static unsigned int factory_kf_id = 0; 
    //     kf_id = factory_kf_id++; 

    //     this->id = base_frame->id; 
    //     this->left_img = base_frame->left_img; 
    //     this->right_img = base_frame->right_img; 
    //     this->features_on_left_img = base_frame->features_on_left_img; 
    //     this->setPose(base_frame->getPose()); 

    //     for(size_t i = 0; i < base_frame->features_on_left_img.size(); i++)
    //     {
    //         auto mp = base_frame->features_on_left_img[i]->map_point.lock();
    //         if(mp != nullptr)
    //         {
    //             features_on_left_img[i]->map_point = mp;
                
    //         }
    //     }
    // }

} //! end of namespace 