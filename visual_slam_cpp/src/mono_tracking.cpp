/**
 * @file mono_tracking.cpp
 * @author mrostocki
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/mono_tracking.hpp" 
#include "mrVSLAM/frame.hpp"
#include "mrVSLAM/visualizer.hpp"
#include "mrVSLAM/camera.hpp"

namespace mrVSLAM
{   
    MonoTracking::MonoTracking(const DetectorType& detector_type, const bool& use_descriptors,
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
    }

    bool MonoTracking::addNewFrame(std::shared_ptr<Frame> frame)
    {
        current_frame = frame; 

        switch (tracking_status)
        {
        case TrackingStatus::INITING:

            if(monoInitialization()){
                this->tracking_status = TrackingStatus::TRACKING; 
                return false; 
            }
            break;
        case TrackingStatus::TRACKING:
            track(); 
            break; 
        case TrackingStatus::LOST:
            return false; 
            break;
        }
        visualizer->addNewFrame(current_frame); 
        prev_frame = current_frame; 
        return true;  
    }

    void MonoTracking::insertKeyframe()
    {

    }

    
    bool MonoTracking::monoInitialization()
    {

        //* set first frame as a reference frame and detect features 
        if(need_for_reference_frame) {
            fmt::print("Mono initialization; setting reference frame");
            need_for_reference_frame = false; 
            this->reference_kf = current_frame; 

            if(this->use_descriptors)
                extractFeatures(); 
            else 
                detectFeatures(); 
            
            current_frame->setFrameToKeyframe(); 
            //if there is only one frame than can't do anything more 
            return false; 
        }

        //get detected points from reference frame and 
        std::vector<cv::Point2f> kps_reference, kps_current; 
        for(auto& ref_features : reference_kf->features_on_left_img){
            
            kps_reference.emplace_back(ref_features->positionOnImg.pt); 
            kps_current.emplace_back(ref_features->positionOnImg.pt);
        }

        // if(prev_frame == nullptr) {fmt::print("chuj\n"); }  
        // //get detected points from prev frame to set initial guesses for flow in current frame 
        // for(auto& prev_features : prev_frame->features_on_left_img){
        //     if(prev_features != nullptr) {
        //         kps_current.emplace_back(prev_features->positionOnImg.pt); 
        //     }
        // }


        //* get position of point by optical flow and create new features 
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK( reference_kf->left_img, current_frame->left_img, kps_reference,
                                  kps_current, status, error, cv::Size(21, 21), 3,
                                  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0; 
        for (size_t i = 0; i < status.size(); ++i) 
        {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                auto new_feature = std::make_shared<Feature>(current_frame,kp, true); // false = is on right img 
                current_frame->features_on_left_img.emplace_back(new_feature);
                num_good_pts++;
            } else {
                current_frame->features_on_left_img.emplace_back(nullptr);
            }
        }

        fmt::print("mono tracking initialization: number of found points with optical flow = {} \n", num_good_pts); 

        if(num_good_pts < 50) {
            need_for_reference_frame = true; 
            fmt::print("mono tracking initialization: not enought points to triangulate \n", num_good_pts); 
            return false; 
        }

        //* estimate movement by essential Matrix 
        std::vector<cv::Point2f> current_good_points, reference_good_points; 
        // assert(current_frame->features_on_left_img.size() == reference_kf->features_on_left_img.size()); 

        for(size_t i = 0; i < current_frame->features_on_left_img.size(); i++)
        {
            if(current_frame->features_on_left_img[i])
            {
                current_good_points.emplace_back(current_frame->features_on_left_img[i]->positionOnImg.pt); 
                reference_good_points.emplace_back(reference_kf->features_on_left_img[i]->positionOnImg.pt); 
            }
        }

        fmt::print("dupa 2 \n");
        cv::Mat K, R, t, inliers, triangulated_points; 
        cv::eigen2cv(camera_left->getK(), K); 
        auto essentialMatrix = cv::findEssentialMat( reference_good_points, current_good_points, K, cv::RANSAC, 0.99, 1.0, 100, inliers); 
        cv::recoverPose(essentialMatrix, reference_good_points, current_good_points, K, R, t, inliers); 

        t = t / sqrt(t.at<double>(1, 0) * t.at<double>(1, 0) + t.at<double>(2, 0) * t.at<double>(2, 0) +
                 t.at<double>(0, 0) * t.at<double>(0, 0));

        Eigen::Matrix3d temp_R; 
        Eigen::Vector3d temp_t; 
        cv::cv2eigen(R, temp_R); 
        cv::cv2eigen(t, temp_t);
        fmt::print("calculated new pose from essentail matrix \n"); 

        Sophus::SE3d temp_pose(temp_R, temp_t); 
        current_frame->setPose(temp_pose); 

        std::cout << "POSE = \n" << temp_pose.matrix3x4() << "\n"; 

        if(std::abs(int(reference_kf->id) - int(current_frame->id)) > 5) 
        {
 



            // createInitialMap(); 
            return true; 
        }


        return false; 
    }

    bool MonoTracking::createInitialMap()
    {
        return true; 
    }

    bool MonoTracking::track()
    {
        return true; 
    }

    /**
     * @brief detect keypoints on image
     * @details create set of masks, detect keypoints and create feature objects assigned to current frame object 
     * @return int number of detected keypoints  
     */
    int MonoTracking::detectFeatures()
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
            auto new_feature = std::make_shared<Feature>(current_frame, kp, true);
            current_frame->features_on_left_img.emplace_back(new_feature);
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
    int MonoTracking::extractFeatures()
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
            auto new_feature = std::make_shared<Feature>(current_frame, keypoints.at(i), descriptors.row(i), true);
            current_frame->features_on_left_img.emplace_back(new_feature); 
            extracted_features++;
        }

        return extracted_features;
    }

} //! end of namespace