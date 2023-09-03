#include "../include/tracking.hpp"
#include "../include/visualizer.hpp"

namespace mrVSLAM
{
    Tracking::Tracking()
    {
        detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance

        // set pointers to map, backend and visualizer 
    }

    void Tracking::setTracking(std::shared_ptr<Map> in_map, std::shared_ptr<Visualizer> in_visualizer, std::shared_ptr<Backend> in_backend)
    {
        map = in_map; 
        visualizer = in_visualizer; 
        backend = in_backend; 
    }

    void Tracking::addFrameAndTrackStereo(std::shared_ptr<Frame> frame_to_add)
    {
        // get frame, set logic 
        current_frame = frame_to_add; 

        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 
                //initialization
                stereoInitialize(); // add choosing which camera case 
                break; 
            case STATUS::TRACKING: 
                track(); 
                break; 
            case STATUS::LOST: 
                restartTracking(); 
                break; 
        }

        prev_frame = current_frame; 
    }

//########################################################

    bool Tracking::initialize()
    {
        // can't initialize stereo without at least 2 frames //? how to get two frames
        if

    }


    bool Tracking::stereoInitialize()
    {
        int num_of_features_in_left_img = detectFeatures(); 
        int num_of_corresponding_features_in_right = findCorrespndingStereoFeatures(); 

        if(num_of_corresponding_features_in_right < num_of_features_init)
            return false; //initialization failed 
        
        if(initializeMap()) //create map and change status to tracking 
        {
            tracking_status = STATUS::TRACKING; 

            if(visualizer != nullptr)
            {
                visualizer->addNewFrame(current_frame); 
                visualizer->getMapUpdate();
            }
            return true; // initiaization succeded 
        }
        return false; //initialization failed  
    }

    void Tracking::track()
    {
        if(prev_frame!=nullptr)
        {
            //if at least 2 frames exist srt current frame pose by multipling transMatrix with pose matrix (homogenous)
            current_frame->SetFramePose(transformationMatrix * prev_frame->getFramePose());
        }
        

        std::vector<cv::Point2f> keypoints_prev_frame, keypoints_current_frame; 
        for(auto &keypoint : prev_frame->featuresFromLeftImg)
        {
            auto mappoint = keypoint->map_point.lock(); //* https://en.cppreference.com/w/cpp/memory/weak_ptr/lock
            keypoints_prev_frame.emplace_back(keypoint->featurePoint_position.pt); 
            keypoints_current_frame.emplace_back(); 
        }

        cv::calcOpticalFlowPyrLK(prev_frame->imgLeft, current_frame->imgLeft, ); 


        //! decision if frame is new keyframe 
        if( /*inliers > num_of_features_for_keyframe */ )
        {
            keyframeInsertion(); 
        }
        
        transformationMatrix = (current_frame->getFramePose() * prev_frame->getFramePose().inverse()); 

        if(visualizer!=nullptr)
        {
            visualizer->addNewFrame(current_frame); 
        }

    }

    void Tracking::keyframeInsertion()
    {
            current_frame->SetFrameToKeyframe(); 
            map->insertKeyFrame(current_frame); 

            detectFeatures(); 
            findCorrespndingStereoFeatures(); 
    }

    unsigned int Tracking::detectFeatures()
    {
        // function detects keypoints in main(left) img and pushes Features to Frame object 
        std::vector<cv::KeyPoint> keypoints; 
        detector->detect(current_frame->imgLeft, keypoints, cv::noArray()); 
        unsigned int detected_features = 0; 

        for(auto &point : keypoints)
        {
            current_frame->featuresFromLeftImg.emplace_back(new Feature{current_frame, point}); 
            detected_features++; 
        }

        return detected_features; 
    }

    unsigned int Tracking::findCorrespndingStereoFeatures()
    {
        std::vector<cv::Point2f> keypoints_left, keypoints_right; 
    }


    bool Tracking::initializeMap()
    {
        // triangulate every corresponding feature points from frame 
        // then based on that create MapPoint 
        // insert this map point to map and connect it to frame 

        //? how to give them id? 

        //map->insertPoint(); 
    }



    void restrtTracking()
    {
        
    }
}