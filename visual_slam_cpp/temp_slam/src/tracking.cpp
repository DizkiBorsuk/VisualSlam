#include "../include/tracking.hpp"

namespace mrVSLAM
{
    Tracking::Tracking()
    {
        detector = cv::GFTTDetector::create(num_of_features, 0.01, 5, false. 0.04); 
    }
    
    bool Tracking::track()
    {
        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 

                break; 
            case STATUS::TRACKING: 

                 break; 
            case STATUS::LOST: 

                break; 
        }
    }
    
    

}