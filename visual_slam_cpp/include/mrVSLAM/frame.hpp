/**
 * @file frame.hpp
 * @author mrostocki 
 * @brief 
 * @version 0.1
 * @date 2024-03-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once 
#include "mrVSLAM/common_includes.hpp" 
#include "DBoW3/DBoW3.h"

namespace mrVSLAM
{
    class MapPoint; 
    class Frame; 
    class KeyFrame; 

    /**
     * @brief class that represents characteristic point observed in img
     */
    class Feature
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::weak_ptr<Frame> keyframe; 
        std::weak_ptr<MapPoint> map_point; 

        cv::KeyPoint positionOnImg; 
        cv::Mat descriptor;  

        bool is_on_left_img = true; 
        bool is_outlier = false;

    public: 
        Feature()=default;  
        
        Feature(const cv::KeyPoint &kp, bool on_left_img = true)
        : positionOnImg(kp), is_on_left_img(on_left_img) {} 

        Feature(const cv::KeyPoint &kp,const cv::Mat &feature_desc, bool on_left_img = true)
        : positionOnImg(kp), descriptor(feature_desc), is_on_left_img(on_left_img) {} 

        Feature(std::shared_ptr<Frame> kf, const cv::KeyPoint &kp, bool on_left_img = true)
        : keyframe(kf), positionOnImg(kp), is_on_left_img(on_left_img) {} 

        Feature(std::shared_ptr<Frame> kf, const cv::KeyPoint &kp,const cv::Mat &feature_desc, bool on_left_img = true)
        : keyframe(kf), positionOnImg(kp), descriptor(feature_desc), is_on_left_img(on_left_img) {} 

    }; 

    /**
     * @class Frame 
     * @brief class that represents current "frame", frame can be understand as infromations about current img, 
     * camera position and orientation in that moment, observed characteristic points and so on. 
     */
    class Frame
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        unsigned int id = 0; 
        unsigned int kf_id = 0; 

        cv::Mat left_img, right_img; ///< camera imgs of the frame 
         
        std::vector<std::shared_ptr<Feature>> features_on_left_img;  ///< vector of ptrs to features found in frame img
        std::vector<std::shared_ptr<Feature>> features_on_right_img; ///< vector of ptrs to features found in frame img

        std::weak_ptr<Frame> prev_kf; ///< ptr to previous keyframe object 
        std::weak_ptr<Frame> loop_kf; ///< ptr to keyframe that was selected by loop closer to be the same keyframe as current one

    public: 
        Frame() = default; 
        Frame(const unsigned int frame_id, const cv::Mat &leftImg, const cv::Mat &rightImg); 

        Sophus::SE3d getPose(); 
        Sophus::SE3d getRelativePose(); 
        void setPose(const Sophus::SE3d &in_pose); 
        void setRelativePose(const Sophus::SE3d &in_relativePose); 

        void setFrameToKeyframe(); 
        void setRelativePoseToLastKf(const Sophus::SE3d &in_pose); 
        void setRelativePoseToLoopKf(const Sophus::SE3d &in_pose);
        Sophus::SE3d getRelativePoseToLastKf(); 
        Sophus::SE3d getRelativePoseToLoopKf(); 

        void setBoW_Vector(DBoW3::BowVector &bow_vec); 
        DBoW3::BowVector getBoW_Vector(); 


    private:     
        //? sophus SE3 objects are initialize by default as identity matrix 
        Sophus::SE3d pose; ///< pose of camera/frame in map 
        Sophus::SE3d relativePose; ///< pose of camera/frame relative to last frame 

        Sophus::SE3d relativePoseToLastKf; 
        Sophus::SE3d relativePoseToLoopKf; ///< 

        DBoW3::BowVector bow_vector; ///< Bag Of Words vector that describes left img based on descriptors, used for loop closing 

        std::mutex frame_mutex; 
    }; 

    // /**
    //  * @brief 
    //  * 
    //  */
    // class KeyFrame : public Frame
    // {
    // public: 
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //     unsigned int kf_id; 

    //     std::weak_ptr<KeyFrame> prev_kf; ///< ptr to previous keyframe object 
    //     std::weak_ptr<KeyFrame> loop_kf; ///< ptr to keyframe that was selected by loop closer to be the same keyframe as current one 
    //     Sophus::SE3d relativePoseToLastKf; 
    //     Sophus::SE3d relativePoseToLoopKf; ///< 
    //     DBoW3::BowVector bow_vector; ///< Bag Of Words vector that describes left img based on descriptors, used for loop closing 

    // public: 
    //     KeyFrame(std::shared_ptr<Frame> base_frame); 
    //     std::vector<cv::KeyPoint> getAllFeaturePoints(); 

    // private: 
    //     std::mutex kf_mutex; 
    // }; 
    
} //! end of namespace 
