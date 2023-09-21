#include "../include/tracking.hpp"
#include "../include/visualizer.hpp"
#include "../include/backend_optimization.hpp"

namespace mrVSLAM
{

    //---- Constructors and main functions ----- //
    Tracking::Tracking(DETECTOR detector_type)
    {
        switch (detector_type)
        {
        case DETECTOR::GFTT:
            detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance
            break;
        case DETECTOR::FAST: 
            detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 
            break; 
        default:
            break;
        }
    }

    Tracking::Tracking(EXTRACTOR extractor_type)
    {
        switch (extractor_type)
        {
        case EXTRACTOR::ORB:
            detector = cv::GFTTDetector::create(num_of_features, 0.01, 20, 3, false, 0.04); //? big min distance
            break;
        case EXTRACTOR::SIFT: 
            detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 
            break; 
        case EXTRACTOR::AKAZE: 
            break; 
        default:
            break;
        }
    }

    void Tracking::setTracking(std::shared_ptr<Map> in_map, std::shared_ptr<Visualizer> in_visualizer, std::shared_ptr<Backend> in_backend, 
                        std::shared_ptr<Camera> in_camera_left, std::shared_ptr<Camera> in_camera_right)
    {
        // set pointers to map, visualizer and backend 
        map = in_map; 
        visualizer = in_visualizer; 
        backend = in_backend; 
        camera_right = in_camera_right; 
        camera_left = in_camera_left; 
    }

    void Tracking::addFrameAndTrackStereo(std::shared_ptr<Frame> frame_to_add)
    {
        // get frame, set logic 
        current_frame = frame_to_add; 

        switch(tracking_status)
        {
            case STATUS::INITIALIZATION: 
                //initialization
                if(stereoInitialize() == true) 
                {  
                    tracking_status = STATUS::TRACKING; 
                }
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
//---- Initialization ----- //

    bool Tracking::initialize()
    {
        // can't initialize stereo without at least 2 frames //? how to get two frames
        

    }

    bool Tracking::stereoInitialize()
    {
        //!DONE
        // initialize stereo tracking 
        int num_of_features_in_left_img = detectFeatures(); 
        int num_of_corresponding_features_in_right = findCorrespondingStereoFeatures(); 

        if(num_of_corresponding_features_in_right < num_of_features_for_initialization)
            return false; //initialization failed 
        
        buildMap(); 
        if(visualizer != nullptr || backend !=nullptr)
        {
            backend->updateMap(); 
            visualizer->addNewFrame(current_frame); 
            visualizer->getMapUpdate();
            return true; // initiaization succeded 
        }
        return false; //initialization failed  
    }

//########################################################
//----  ----- //

    unsigned int Tracking::detectFeatures()
    {
        //!DONE
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

    unsigned int Tracking::findCorrespondingStereoFeatures()
    {
        //!DONE
        /*
            find in rigth image features corresponding to features detected in left img using LK optical flow 
        */
        std::vector<cv::Point2f> keypoints_left, keypoints_right; 
        for(auto &point :current_frame->featuresFromLeftImg)
        {
            keypoints_left.emplace_back(point->featurePoint_position); 
            auto ptr_to_mappoint = point->map_point.lock(); 

            if(ptr_to_mappoint != nullptr)
            {
                //if observed point already exist in map then project this point from world to image and use it as initial guess 
                auto projected_point = camera_right->world2pixel(ptr_to_mappoint->position, current_frame->getFramePose()); 
                keypoints_right.emplace_back(projected_point[0], projected_point[1]); 
            } 
            else
            {
                //if map doesn't exit (initialization) that use the same pixel as in left image 
                keypoints_right.emplace_back(point->featurePoint_position.pt);
            } 
        }

        std::vector<uchar> status; // output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
        cv::Mat err; 
        cv::calcOpticalFlowPyrLK(current_frame->imgLeft, current_frame->imgRight, keypoints_left, keypoints_right, status, err, cv::Size(11,11), 3, 
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01), cv::OPTFLOW_USE_INITIAL_FLOW); 
        unsigned int foundCorrespondences = 0; 
        for(int i = 0; i < status.size(); i++)
        {
            if(status[i] == 1)
            {
                cv::KeyPoint keypoint_in_right(keypoints_right[i], 7); // keypoint pos and size 
                current_frame->featuresFromRightImg.emplace_back(new Feature(current_frame, keypoint_in_right)); 
                foundCorrespondences++; 
            }
            else
            {
                current_frame->featuresFromRightImg.emplace_back(nullptr); 
            }
        }
        return foundCorrespondences; 
    }

    void Tracking::buildMap()
    {
        //!DONE
        /* 
        create initial map 
        
        triangulate every corresponding feature points from frame 
         then based on that create MapPoint,  // ? how to give them id? 
         insert this map point to map and connect it to frame 
        */
        std::vector<Eigen::Vector3d> left_right_featurePoints_in_camera; // detected keypoints in camera coordinate system for triangulation
        unsigned int number_of_points_in_map = 0; 

        for(int i =0; i < current_frame->featuresFromLeftImg.size(); i++)
        {
            //convert feature points to camera coordinate system
            left_right_featurePoints_in_camera.emplace_back(camera_left->pixel2camera(current_frame->featuresFromLeftImg[i]->featurePoint_position, 1)); 
            left_right_featurePoints_in_camera.emplace_back(camera_left->pixel2camera(current_frame->featuresFromRightImg[i]->featurePoint_position, 1)); 
            Eigen::Vector3d point_in_3D = Eigen::Vector3d::Zero(); 

            bool triSuccess = triangulate(left_right_featurePoints_in_camera, camera_left->Rt, camera_right->Rt, point_in_3D); //triangulate features to get point in 3d

            if(triSuccess == true && point_in_3D[2] > 0 ) // check if Z is greater than O to eliminate points "behind" camera 
            {
                auto new_mappoint = std::shared_ptr<MapPoint>(new MapPoint(MapPoint::mappoint_counter, point_in_3D)); //created new map point object 
                MapPoint::mappoint_counter++; // mappoint id counter 
                number_of_points_in_map++; 

                new_mappoint->addFeature(current_frame->featuresFromLeftImg[i]); 
                new_mappoint->addFeature(current_frame->featuresFromRightImg[i]); 

                current_frame->featuresFromLeftImg[i]->map_point = new_mappoint; 
                current_frame->featuresFromRightImg[i]->map_point = new_mappoint; 

                map->insertPoint(new_mappoint);
            }
        }
        std::cout << "Map created with " << number_of_points_in_map << "point in map \n"; 

        current_frame->SetFrameToKeyframe(); 
        map->insertKeyFrame(current_frame); 
    }


    void Tracking::track()
    {
        //? in work 
        if(prev_frame!=nullptr){
            //if at least 2 frames exist set current frame pose by multipling transMatrix with pose matrix (homogenous)
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

        //! decision if current frame is new keyframe 
        if( /*inliers > num_of_features_for_keyframe */ )
        {
            newKeyframeInsertion(); 
        }
        
        transformationMatrix = (current_frame->getFramePose() * prev_frame->getFramePose().inverse()); 

        if(visualizer!=nullptr)
        {
            visualizer->addNewFrame(current_frame); 
        }

    }

    void Tracking::newKeyframeInsertion()
    {
        //? in work 
        /*
        make current frame a keyframe, insert keyframe to map and visualizer, add 
        */
        current_frame->SetFrameToKeyframe(); 
        std::cout << "new keyframe id = " << current_frame->keyframe_id << "\n"; 
        map->insertKeyFrame(current_frame); 



        detectFeatures();
        findCorrespondingStereoFeatures();
        createNewMapPoints(); 

        // add features from keyframe to observed mappoints
        for(auto &feature : current_frame->featuresFromLeftImg)
        {
            auto mappoint = feature->map_point.lock(); 
            if(mappoint != nullptr)
            {
                mappoint->addFeature(feature); 
            }
        }

        std::cout << " added keyframe \n"; 

        detectFeatures(); 

        findCorrespondingStereoFeatures(); 
    }

    void Tracking::createNewMapPoints()
    {
        /*
        triangulate new map points 
        */

       Eigen::Matrix4d currentPose_Twc = current_frame->framePose.inverse(); //camera to world Tra
       unsigned int number_of_triangulatedPoints = 0; 

       for(int i  = 0; i < current_frame->featuresFromLeftImg.size(); i++)
       {
            if(current_frame->featuresFromLeftImg[i]->map_point.expired() )
            {
                
            }
       }
    }


    void Tracking::restartTracking()
    {
        
    }
}