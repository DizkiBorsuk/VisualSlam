#pragma once 
#include "common_includes.hpp"


namespace mrVSLAM
{
    class Frame
    {
    public: 
        //### data members ###//
        unsigned int id = 0; // id of a frame 
        unsigned int keyframe_id = 0; // id of a frame if it's a keyframe 
        bool is_keyframe = false; // setting if frame is a keyframe 
        cv::Matx44d framePose = cv::Matx44::eye(); // pose of a frame 

        std::mutex poseMutex; // mutex for pose to get and set it thread safetly 
        std::vector<std::shared_ptr<Feature>> featuresFromLeftImg; 
        std::vector<std::shared_ptr<Feature>> featuresFromRightImg;

        //### member functions ####// 

        Frame(unsigned int id, const cv::Matx44d &pose, const cv::Mat &img_left, const cv::Mat &img_right) noexcept; 

        cv::Matx44d getFramePose();  // get frame pose from thread 
        void SetFramePose(const cv::Matx44d &pose);  // set frame pose in thread 
        void SetFrameToKeyFrame(); 
        
        static std::shared_ptr<Frame> createFrame(); 

    private: 
        static unsigned int tem_keyframe_id = 0; 

    }; 



}


