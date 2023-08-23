#include "../include/tracking.hpp"

namespace mrVSLAM
{
    Tracking::Tracking()
    {
        detector = cv::GFTTDetector::create(num_of_features, 0.01, 5, 3, false, 0.04); 
    }
    
    void Tracking::addFrameAndTrack(std::shared_ptr<Frame> frame_to_add)
    {
        current_frame = frame_to_add; 

        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 

                break; 
            case STATUS::TRACKING: 
                track(); 
                break; 
            case STATUS::LOST: 

                break; 
        }

        prev_frame = current_frame; 
    }

    void Tracking::stereoInitialize()
    {
        int num_of
    }

    void Tracking::track()
    {
        if(prev_frame!=nullptr)
        {
            current_frame->SetFramePose(transformationMatrix * prev_frame->getFramePose());
        }
             
        


    }
    
    

}