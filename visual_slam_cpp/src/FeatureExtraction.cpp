#include "../include/FeatureExtraction.hpp"


void mrVSLAM::FeatureExtraction::getFeatures(cv::Mat frame, const desctiptor_T& descriptor_type)
{
    switch(descriptor_type)
    {
        case sift: 
            detector = cv::SIFT::create(num_features);  
            descriptor = cv::SIFT::create(num_features); 
            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        case orb: 
            detector = cv::ORB::create(num_features);  
            descriptor = cv::ORB::create(num_features); 
            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        case akaze: 
            detector = cv::AKAZE::create(); 
            descriptor = cv::AKAZE::create(); 

            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        default: 
            break;  

    }
}

void mrVSLAM::FeatureExtraction::matchFeaturesFlann(const float& low_rt)
{    
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
    descriptors.convertTo(descriptors, CV_32F);

    std::vector<std::vector<cv::DMatch>> matches;
    cv::Point2i keypoint1, keypoint2; 
    std::vector<cv::Point2i> point_pair;  

    if(!prev_descriptors.empty())
    {
        matcher->knnMatch(descriptors, prev_descriptors, matches, 2); 

        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches[i][0].distance < low_rt * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);
                keypoint1 = keypoints[matches[i][0].queryIdx].pt;  
                keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
                point_pair.push_back(keypoint1); 
                point_pair.push_back(keypoint2); 
                matched_keypoints.push_back(point_pair); 
            }
           point_pair.clear(); 
        }
    }
    prev_descriptors = descriptors; 
    prev_keyPs = keypoints; 
}

void mrVSLAM::FeatureExtraction::matchFeaturesBF(const float& low_rt)
{    
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 

    std::vector<std::vector<cv::DMatch>> matches;
    cv::Point2i keypoint1, keypoint2; 
    std::vector<cv::Point2i> point_pair; 

    if(!prev_descriptors.empty())
    {
        matcher->knnMatch(descriptors, prev_descriptors, matches, 2); 

        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches[i][0].distance < low_rt * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);
                keypoint1 = keypoints[matches[i][0].queryIdx].pt;  
                keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
                point_pair.push_back(keypoint1); 
                point_pair.push_back(keypoint2); 
                matched_keypoints.push_back(point_pair); 
            }
           point_pair.clear(); 
        }
    }
    prev_descriptors = descriptors; 
    prev_keyPs = keypoints; 
}


void mrVSLAM::FeatureExtraction::getFeatures(cv::cuda::GpuMat frame, const desctiptor_T& descriptor_type)
{
    switch(descriptor_type)
    {
        case sift:  
            break; 
        case orb: 
            gpu_orb_extractor = cv::cuda::ORB::create(num_features,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);
            gpu_orb_extractor->detectAndCompute(frame,cv::noArray(), keypoints, gpu_descriptor); 
            break; 
        case akaze: 
            break; 
        default: 
            break;  
    }

        // cv::Ptr<cv::cuda::ORB> gpu_detector = cv::cuda::ORB::create(num_features,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);  
        // //gpu_ORB->detectAndComputeAsync(frame, cv::noArray(), gpu_keypoints_1, gpu_descriptors_1, false); 
        // //gpu_detector->detectAndCompute(frame, cv::noArray(), keypoints_1, gpu_descriptors_1, false); 
        
        // gpu_detector->detect(frame, keypoints_1);
        // std::cout << "Number of features: " << keypoints_1.size() << "\n"; 
        // //gpu_ORB->compute(frame, gpu_keypoints_1, descriptors_1_gpu); 
        // //gpu_ORB->convert(gpu_keypoints_1, keypoints_1); 
}
