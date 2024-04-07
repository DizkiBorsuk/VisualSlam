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
            monoInitialize(); 
            break;
        case TrackingStatus::TRACKING:
            track(); 
            break; 
        case TrackingStatus::LOST:
            return false; 
            break;
        }

        if(visualizer) { visualizer->addNewFrame(current_frame); } 
        
        prev_frame = current_frame; 
        return true;  
    }

    void MonoTracking::insertKeyframe()
    {

    }

    
    bool MonoTracking::monoInitialize()
    {
        return false; 
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