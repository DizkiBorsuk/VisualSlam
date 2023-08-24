#include "../include/tracking.hpp"

namespace mrVSLAM
{
    Tracking::Tracking()
    {
        detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance
    }
    
    void Tracking::addFrameAndTrack(std::shared_ptr<Frame> frame_to_add)
    {
        current_frame = frame_to_add; 

        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 
                stereoInitialize(); // add 
                break; 
            case STATUS::TRACKING: 
                track(); 
                break; 
            case STATUS::LOST: 
                RestartTracking(); 
                break; 
        }

        prev_frame = current_frame; 
    }

    void Tracking::stereoInitialize()
    {
        int num_of_features_in_left_img = detectFeatures(); 
        int num_of_corresponding_features_in_right = findCorrFeatures(); 
    }

    void Tracking::track()
    {
        if(prev_frame!=nullptr)
        {
            //if at least 2 frames exist srt current frame pose by multipling transMatrix with pose matrix (homogenous)
            current_frame->SetFramePose(transformationMatrix * prev_frame->getFramePose());
        }
             
        


    }

    unsigned int Tracking::detectFeatures()
    {
        std::vector<cv::KeyPoint> keypoints; 
        detector->detect(current_frame->imgLeft, keypoints, cv::noArray()); 
        unsigned int detected_features = 0; 

        for(auto &point : keypoints)
        {
            current_frame->featuresFromLeftImg.emplace_back(new Feature{current_frame, point}); 
            detected_features++; 
        }

    }
    
    

}