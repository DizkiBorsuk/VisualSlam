#include "../include/FrameExtraction.hpp"

namespace mrVSLAM 
{
    Frame::Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures)
    {
        this->frameId = frameId; 
        K = cameraMatrix; 
        invK = K.inv(); 
        pose = pose.eye(); 

        auto features = extraxtFeatures(image, ExtractorType::OrbFast, numOfFeatures); 
        frameFeaturePoints = features.frameKeypoints; 
        frameDescriptors = features.descriptors; 
        /*
        add coordinates normalization, transform from img cordinates to camera coordinates 
        */
    }

    feature extraxtFeatures(const cv::Mat &img, const ExtractorType extractor, const int numberOfFeatures)
    {
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptor; 
        std::vector<cv::KeyPoint> keypoints; 
        cv::Mat descriptors; 

        keypoints.reserve(numberOfFeatures);

        switch (extractor)
        {
        case ExtractorType::OrbHarris:
            detector = cv::ORB::create(numberOfFeatures,cv::ORB::HARRIS_SCORE);  
            descriptor = cv::ORB::create(numberOfFeatures, cv::ORB::HARRIS_SCORE);
            break;
        case ExtractorType::OrbFast:
            detector = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE);  
            descriptor = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE); 
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
        detector->detect(img, keypoints); 
        descriptor->compute(img, keypoints, descriptors); 

        std::cout << "descriptors size" << descriptors.size()<< "\n"; 
        return {keypoints, descriptors}; 
    }    
    
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures)  
    {   
        cv::cuda::GpuMat descriptors, keypoints; 
        //cv::Ptr<cv::cuda::FastFeatureDetector> gpuFastDetector; 
        cv::Ptr<cv::cuda::ORB> gpuOrbExtractor = cv::cuda::ORB::create(500,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);
        gpuOrbExtractor->detectAndComputeAsync(img,cv::noArray(), keypoints, descriptors, false); 
        // //gpu_ORB->detectAndComputeAsync(frame, cv::noArray(), gpu_keypoints_1, gpu_descriptors_1, false); 
        
        // gpu_detector->detect(frame, keypoints_1);
        // //gpu_ORB->compute(frame, gpu_keypoints_1, descriptors_1_gpu); 

        return {keypoints, descriptors}; 
    }

    // void FrameMatcher::matchFeaturesFlann(const float& low_rt) noexcept
    // {    
    //     matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
    //     descriptors.convertTo(descriptors, CV_32F);

    //     std::vector<std::vector<cv::DMatch>> matches;
    //     cv::Point2f keypoint1, keypoint2; 
    //     std::vector<cv::Point2f> point_pair;  

    //     if(!prev_descriptors.empty())
    //     {
    //         matcher->knnMatch(descriptors, prev_descriptors, matches, 2); // finds 2 best matches for descriptors 

    //         for (size_t i = 0; i < matches.size(); i++)
    //         {
    //             if (matches[i][0].distance < low_rt * matches[i][1].distance) // compare 1 best match with second best match  
    //             {
    //                 good_matches.emplace_back(matches[i][0]);
    //                 keypoint1 = frame_keypoints[matches[i][0].queryIdx].pt;  
    //                 keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
    //                 point_pair.emplace_back(keypoint1); 
    //                 point_pair.emplace_back(keypoint2); 
    //                 matched_keypoints.emplace_back(point_pair); 
    //             }
    //             point_pair.clear(); 
    //         }
    //     }
    //     prev_descriptors = descriptors; 
    //     prev_keyPs = frame_keypoints; 
    // }

    // void FrameMatcher::matchFeaturesBF(const float& low_rt) noexcept
    // {    
    //     matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 

    //     std::vector<std::vector<cv::DMatch>> matches;
    //     cv::Point2f keypoint1, keypoint2; 
    //     std::vector<cv::Point2f> point_pair; 

    //     if(!prev_descriptors.empty())
    //     {
    //         matcher->knnMatch(descriptors, prev_descriptors, matches, 2); 

    //         for (size_t i = 0; i < matches.size(); i++)
    //         {
    //             if (matches[i][0].distance < low_rt * matches[i][1].distance)
    //             {
    //                 good_matches.emplace_back(matches[i][0]);
    //                 keypoint1 = frame_keypoints[matches[i][0].queryIdx].pt;  
    //                 keypoint2 = prev_keyPs[matches[i][0].trainIdx].pt; 
    //                 point_pair.emplace_back(keypoint1); 
    //                 point_pair.emplace_back(keypoint2); 
    //                 matched_keypoints.emplace_back(point_pair); 

    //             }
    //         point_pair.clear(); 
    //         }
    //     }
    //     prev_descriptors = descriptors; 
    //     prev_keyPs = frame_keypoints; 
    // }
}
