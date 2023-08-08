#include "../include/FrameExtraction.hpp"

namespace mrVSLAM 
{
    Frame::Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures, FeatureExtractor *extractor) noexcept
                : frameId(frameId), K(cameraMatrix)
    {
        frameFeaturePoints.reserve(numOfFeatures+100); 
        extractor->extractFeatures(image, frameFeaturePoints, frameDescriptors); 
        /*
        add coordinates normalization, transform from img cordinates to camera coordinates 
        */
    }

    //####################
    FeatureExtractor::FeatureExtractor(const ExtractorType &extractor, const int &numberOfFeatures) noexcept
    {
        switch (extractor)
        {
        case ExtractorType::OrbHarris:
            detector = cv::ORB::create(numberOfFeatures,cv::ORB::HARRIS_SCORE);  
            descriptor = cv::ORB::create(numberOfFeatures, cv::ORB::HARRIS_SCORE);
            break;
        case ExtractorType::ORB:
            detector = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE);  
            descriptor = cv::ORB::create(numberOfFeatures, 1.200000048F, 8, 31, 0, 2, cv::ORB::FAST_SCORE); 
            break; 
        case ExtractorType::OrbFast:
            detector = cv::FastFeatureDetector::create(40, true, cv::FastFeatureDetector::TYPE_9_16); 
            descriptor = cv::ORB::create(numberOfFeatures);
            break;
        case ExtractorType::OrbGptt:
            detector = cv::GFTTDetector::create(numberOfFeatures, 0.01, 1.0, 3, false, 0.04); 
            descriptor = cv::ORB::create(numberOfFeatures, cv::ORB::FAST_SCORE);
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

    void FeatureExtractor::extractFeatures(const cv::Mat &img, std::vector<cv::KeyPoint> &outKeypoint, cv::Mat &outDescriptors) noexcept
    {
        detector->detect(img, outKeypoint); 
        descriptor->compute(img, outKeypoint, outDescriptors); 
    }   
    
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures) noexcept
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

    //###################   

    FrameMatcher::FrameMatcher(MatcherType matcherType) noexcept
    {
        switch (matcherType)
        {
        case Flann:
            matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
            descT = float32;  
            break;
        case BruteForce: 
            matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 
            descT = uchar8;  
            break; 
        default:
            break;
        }
    }

    void FrameMatcher::matchFrames(const Frame &frame1, const Frame &frame2, const float& low_ratio) noexcept
    {
        std::vector<std::vector<cv::DMatch>> matches; 
        switch (descT)
        {
        case float32:
                frame1.frameDescriptors.convertTo(descriptors1, CV_32F); 
                frame2.frameDescriptors.convertTo(descriptors2, CV_32F);
                matcher->knnMatch(descriptors1, descriptors2, matches, 2); 
            break;
        case uchar8:
            matcher->knnMatch(frame1.frameDescriptors,frame2.frameDescriptors, matches, 2);  
            break; 
        
        default:
            break;
        }

        std::vector<std::vector<cv::DMatch>>::iterator it;
        for (it= matches.begin(); it!= matches.end(); ++it) 
        {
            if ((*it)[0].distance/(*it)[1].distance < low_ratio)
            {
                goodMatches.emplace_back((*it)[0]);
                keypoint1 = frame1.frameFeaturePoints[(*it)[0].queryIdx].pt;  
                keypoint2 = frame2.frameFeaturePoints[(*it)[0].trainIdx].pt; 
                pointPair[0] = keypoint1; 
                pointPair[1] = keypoint2; 
                matchedKeypoints.emplace_back(pointPair); 
            }
        }
    }

}
