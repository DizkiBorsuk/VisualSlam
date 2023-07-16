#include "../include/FeatureExtraction.hpp"
#include "../include/Ransac.hpp"

namespace mrVSLAM 
{
    FeatureExtraction::FeatureExtraction(const ExtractorType extractor, const bool GPU, const int numberOfFeatures) noexcept
    {
        /* Feature extraction class constructor
        This constructor creates detector and descriptor objects based on specified descriptor type.
        */

        if(GPU == false)
        {
            switch (extractor)
            {
            case ExtractorType::OrbHarris:
                detector = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31,20);  
                descriptor = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31,20);
                break;
            case ExtractorType::OrbFast:
                detector = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31,40);  
                descriptor = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31,40); 
                break; 
            case ExtractorType::ORB:
                detector = cv::FastFeatureDetector::create(40); 
                descriptor = cv::ORB::create(numberOfFeatures);
                break;
            case ExtractorType::OrbGptt:
                detector = cv::GFTTDetector::create(); 
                descriptor = cv::ORB::create(numberOfFeatures);
                break;
            case ExtractorType::SIFT:
                detector = cv::SIFT::create(numberOfFeatures);  
                descriptor = cv::SIFT::create(numberOfFeatures); 
                break;
            case ExtractorType::AKAZE:
                detector = cv::AKAZE::create(); 
                descriptor = cv::AKAZE::create(); 
                break;            
            default: 
                
                break;
            }
        } 
        else 
        {
            // add gpu detecotr declaration 
        }

        frame_keypoints.reserve(numberOfFeatures); // change to double from int 
    }
    

    void FeatureExtraction::getFeatures(const cv::Mat frame) noexcept
    {
        detector->detect(frame, frame_keypoints); 
        descriptor->compute(frame, frame_keypoints, descriptors); 
    }

    void FeatureExtraction::getGPUFeatures(const cv::cuda::GpuMat frame) noexcept
    {
        gpuOrbExtractor = cv::cuda::ORB::create(500,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);
        gpuOrbExtractor->detectAndComputeAsync(frame,cv::noArray(), gpuDescriptors, gpuKeypoints, false); 
        // cv::Ptr<cv::cuda::ORB> gpu_detector = cv::cuda::ORB::create(num_features,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);  
        // //gpu_ORB->detectAndComputeAsync(frame, cv::noArray(), gpu_keypoints_1, gpu_descriptors_1, false); 
        // //gpu_detector->detectAndCompute(frame, cv::noArray(), keypoints_1, gpu_descriptors_1, false); 
        
        // gpu_detector->detect(frame, keypoints_1);
        // std::cout << "Number of features: " << keypoints_1.size() << "\n"; 
        // //gpu_ORB->compute(frame, gpu_keypoints_1, descriptors_1_gpu); 
        // //gpu_ORB->convert(gpu_keypoints_1, keypoints_1); 
    }

    void FrameMatcher::matchFeaturesFlann(const float& low_rt) noexcept
    {    
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
        descriptors.convertTo(descriptors, CV_32F);

        std::vector<std::vector<cv::DMatch>> matches;
        cv::Point2f keypoint1, keypoint2; 
        std::vector<cv::Point2f> point_pair;  

        if(!prev_descriptors.empty())
        {
            matcher->knnMatch(descriptors, prev_descriptors, matches, 2); // finds 2 best matches for descriptors 

            for (size_t i = 0; i < matches.size(); i++)
            {
                if (matches[i][0].distance < low_rt * matches[i][1].distance) // compare 1 best match with second best match  
                {
                    good_matches.emplace_back(matches[i][0]);
                    keypoint1 = frame_keypoints[matches[i][0].queryIdx].pt;  
                    keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
                    point_pair.emplace_back(keypoint1); 
                    point_pair.emplace_back(keypoint2); 
                    matched_keypoints.emplace_back(point_pair); 
                }
                point_pair.clear(); 
            }
        }
        prev_descriptors = descriptors; 
        prev_keyPs = frame_keypoints; 
    }

    void FrameMatcher::matchFeaturesBF(const float& low_rt) noexcept
    {    
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 

        std::vector<std::vector<cv::DMatch>> matches;
        cv::Point2f keypoint1, keypoint2; 
        std::vector<cv::Point2f> point_pair; 

        if(!prev_descriptors.empty())
        {
            matcher->knnMatch(descriptors, prev_descriptors, matches, 2); 

            for (size_t i = 0; i < matches.size(); i++)
            {
                if (matches[i][0].distance < low_rt * matches[i][1].distance)
                {
                    good_matches.emplace_back(matches[i][0]);
                    keypoint1 = frame_keypoints[matches[i][0].queryIdx].pt;  
                    keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
                    point_pair.emplace_back(keypoint1); 
                    point_pair.emplace_back(keypoint2); 
                    matched_keypoints.emplace_back(point_pair); 

                }
            point_pair.clear(); 
            }
        }
        prev_descriptors = descriptors; 
        prev_keyPs = frame_keypoints; 
    }
}
