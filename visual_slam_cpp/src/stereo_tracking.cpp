/**
 * @file stereo_tracking.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/stereo_tracking.hpp" 
#include "mrVSLAM/tools.hpp"
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/local_mapping.hpp"
#include "mrVSLAM/map.hpp"
#include "mrVSLAM/mappoint.hpp"
#include "mrVSLAM/loop_closing.hpp"
#include "mrVSLAM/camera.hpp"
#include "mrVSLAM/visualizer.hpp"

namespace mrVSLAM
{   
    //! done 
    StereoTracking::StereoTracking(const DetectorType& detector_type, const bool& use_descriptors,
                                   const unsigned int& num_features)
    {
        this->detector_type = detector_type; 
        this->use_descriptors = use_descriptors; 
        this->num_features = num_features; 

        switch(this->detector_type)
        {
            case DetectorType::GFTT: 
                detector = cv::GFTTDetector::create(this->num_features, 0.01, 10, 3, false, 0.04); 
                if(this->use_descriptors)
                    extractor = cv::ORB::create(this->num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
                break; 
            case DetectorType::ORB: 
                detector = cv::ORB::create(this->num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); // WTA_K can be change to 3 and 4 but then BRUTEFORCE_HAMMING myst be changed to BRUTEFORCE_HAMMINGLUT
                if(this->use_descriptors) 
                    extractor = cv::ORB::create(this->num_features, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE); 
                break; 
            case DetectorType::SIFT: 
                detector = cv::SIFT::create(this->num_features, 3, 0.04, 10, 1.6, false);  
                if(this->use_descriptors) 
                    extractor = cv::SIFT::create(this->num_features, 3, 0.04, 10, 1.6, false); 
                break;
            case DetectorType::SUPER_POINT: 
                break; 
        }
        fmt::print(fg(fmt::color::aqua), "stereo tracking initialized \n"); 

    } // end of StereoTracking::StereoTracking

    //!done 
    bool StereoTracking::addNewFrame(std::shared_ptr<Frame> frame)
    {
        current_frame = frame; 

        switch (tracking_status)
        {
        case TrackingStatus::INITING:
            stereoInitialize(); 
            break;
        case TrackingStatus::TRACKING:
            track(); 
            break; 
        case TrackingStatus::LOST:
            return false; 
            break;
        }

        //if(visualizer) { visualizer->addNewFrame(current_frame); } 
        
        prev_frame = current_frame; 
        return true; 
    } //End of StereoTracking::addNewFrame

    
    /** 
     * @brief initialization of map for stereo tracking  
     * @details 
     * @return true 
     * @return false 
     */ 
    bool StereoTracking::stereoInitialize()
    {
        auto beginINIT = std::chrono::steady_clock::now();

        if(this->use_descriptors)
            extractFeatures(); 
        else 
            detectFeatures(); 
        
        auto found_point_pairs = findCorrespondingPoints();
        fmt::print("stereoInitialize: found {} point pairs \n", found_point_pairs); 

        if(found_point_pairs < this->num_features_init)
            return false; 
        
        auto map_created = createInitialMap(); 

        if(map_created == true)
        {
            //* set frame to keyframe */ 
            current_frame->setFrameToKeyframe(); 

            if(map) { map->insertNewKeyframe(current_frame); }
            if(local_mapping) { local_mapping->updateMap(); }
            if(loop_closer) { loop_closer->insertKeyframe(current_frame); }
            
            visualizer->addNewFrame(current_frame); 
            visualizer->updateMap(); 

            this->tracking_status = TrackingStatus::TRACKING; 
        } else {
            return false; 
        }
        
        auto endINIT = std::chrono::steady_clock::now();
        auto elapsedINIT = std::chrono::duration_cast<std::chrono::milliseconds>(endINIT - beginINIT);
        fmt::print(fg(fmt::color::yellow), "stereo tracking initialization time =  {} ms \n", elapsedINIT.count()); 
        return true; 
    } //* end of StereoTracking::stereoInitialize()

    /**
     * @brief function that creates initial map
     * @details 
     * @return true 
     * @return false 
     */ //!i think it's done 
    bool StereoTracking::createInitialMap()
    {
        unsigned int created_landmarks = 0; 

        for(size_t i_ft = 0; i_ft < current_frame->features_on_left_img.size(); i_ft++)
        {
            if(current_frame->features_on_right_img.at(i_ft) == nullptr) {
                continue;
            }

            std::vector<Eigen::Vector3d> cam_points {camera_left->pixel2camera(convertToVec(current_frame->features_on_left_img.at(i_ft)->positionOnImg.pt)), 
                                                     camera_right->pixel2camera(convertToVec(current_frame->features_on_right_img.at(i_ft)->positionOnImg.pt))
                                                    }; 
            Eigen::Vector3d point_world = Eigen::Vector3d::Zero(); 

            if(triangulate(camera_left->getPose(), camera_right->getPose(), cam_points, point_world))
            {
                auto new_map_point = std::shared_ptr<MapPoint>(new MapPoint(point_world)); 
                new_map_point->addObservation(current_frame->features_on_left_img.at(i_ft)); //! test if needed 
                new_map_point->addObservation(current_frame->features_on_right_img.at(i_ft)); 
                current_frame->features_on_left_img.at(i_ft)->map_point = new_map_point; 
                current_frame->features_on_right_img.at(i_ft)->map_point = new_map_point; 

                map->insertNewMappoint(new_map_point); 
                created_landmarks++; 
            }
        }

        if(created_landmarks>this->num_features_init) {
            return true; 
        }

        return false; 
    } //* end of StereoTracking::createInitialMap()


    bool StereoTracking::track()
    {
        auto beginTrack = std::chrono::steady_clock::now();


        return false; 
    } 
    int StereoTracking::trackLastFrame()
    {
        return 0; 
    }
    void StereoTracking::estimateCurrentPose()
    {

    }


    //?
    /**
     * @brief 
     * @details 1. create new keyframe object 
     * 2. check if reference keyframe exist 
     * 3. if it does, set relative pose to it 
     * 4 if not continue 
     * 5. pass keyframe to all modules 
     */
    void StereoTracking::insertKeyframe()
    {    
        /* ################################# */
        // first part - detect new features and triangulate points 
        for (auto &feat : current_frame->features_on_left_img) 
        {
            auto mp = feat->map_point.lock();
            if (mp) mp->addObservation(feat);
        }
        
        if(use_descriptors == true){
            extractFeatures(); 
        } else {
            detectFeatures();
        }
        // track in right image
        findCorrespondingPoints();
        // triangulate map points
        triangulateNewPoints();

        /* ################################# */
        // second part - create keyframe object and pase it to map and other modules 

        if(reference_kf != nullptr)
        {
            // set connection between two keyframes 
            new_keyframe->prev_kf = reference_kf; 
            //new_keyframe->setRelativePose(); 
        } 

        reference_kf = new_keyframe; 

        map->insertNewKeyframe(new_keyframe); 
        local_mapping->updateMap(); 
        visualizer->updateMap(); 

        if(loop_closer)
            loop_closer->insertKeyframe(new_keyframe); 
        
    } // End of StereoTracking::insertKeyframe

  


    int StereoTracking::triangulateNewPoints()
    {
        return 0; 
    }


    /**
     * @brief try to find points in right img that corresponds to found points in left point (only for keyframe)
     * @details function uses lucas-kanade optical flow to find corresponding points, then creates feature objects for every found points pair
     * and adds it to current frame 
     * @return int 
     */ //!done 
    int StereoTracking::findCorrespondingPoints()
    {
        int num_good_pts = 0;

        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame->features_on_left_img) 
        {
            kps_left.emplace_back(kp->positionOnImg.pt);
            auto mp = kp->map_point.lock();
            if (mp) {
                // use projected points as initial guess
                auto px = camera_right->world2pixel(mp->getPointPosition(), current_frame->getPose());
                kps_right.emplace_back(cv::Point2f(px[0], px[1]));
            } else {
                // use same pixel in left iamge
                kps_right.emplace_back(kp->positionOnImg.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK( current_frame->left_img, current_frame->right_img, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        for (size_t i = 0; i < status.size(); ++i) 
        {
            if (status[i]) {
                cv::KeyPoint kp(kps_right[i], 7);
                std::shared_ptr<Feature> feat(new Feature(kp));
                feat->is_on_left_img = false;
                current_frame->features_on_right_img.emplace_back(feat);
                num_good_pts++;
            } else {
                current_frame->features_on_right_img.emplace_back(nullptr);
            }
        }

        std::cout  << "Find " << num_good_pts << " in the right image. \n";
        return num_good_pts;
    }

    /**
     * @brief detect keypoints on image
     * @details create set of masks, detect keypoints and create feature objects assigned to current frame object 
     * @return int number of detected keypoints  
     */
    int StereoTracking::detectFeatures()
    {
        cv::Mat mask(current_frame->left_img.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_on_left_img) 
        {
            cv::rectangle(mask, feat->positionOnImg.pt - cv::Point2f(10, 10), feat->positionOnImg.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat spf_descriptors; 
        keypoints.reserve(num_features); 

        detector->detect(current_frame->left_img, keypoints, mask);

        unsigned int detected_features = 0;
        for (auto &kp : keypoints) 
        {
            current_frame->features_on_left_img.emplace_back(new Feature(kp));
            detected_features++;
        }

        std::cout  << "Detect " << detected_features << " new features \n";
        return detected_features; 
    }
    
    /**
     * @brief extract features (keypoint + descriptor)
     * @details create set of masks, detect keypoints, create descriptors and create feature objects assigned to current frame object
     * @return int 
     */
    int StereoTracking::extractFeatures()
    {
        cv::Mat mask(current_frame->left_img.size(), CV_8UC1, 255);
        for (auto &feat : current_frame->features_on_left_img) 
        {
            cv::rectangle(mask, feat->positionOnImg.pt - cv::Point2f(10, 10), feat->positionOnImg.pt + cv::Point2f(10, 10), 0, cv::FILLED); 
        }

        unsigned int extracted_features = 0;

        std::vector<cv::KeyPoint> keypoints;
        keypoints.reserve(num_features); 
        cv::Mat descriptors; // each row is different descriptor 

        detector->detect(current_frame->left_img, keypoints, mask);  
        extractor->compute(current_frame->left_img, keypoints, descriptors); 
        // vocabulary->transform(descriptors, current_frame->bow_vector); 

        for(size_t i = 0; i < keypoints.size(); i++)
        {
            current_frame->features_on_left_img.emplace_back(new Feature(keypoints.at(i), descriptors.row(i))); 
            extracted_features++;
        }

        return extracted_features;
    }

} //! end of namespace