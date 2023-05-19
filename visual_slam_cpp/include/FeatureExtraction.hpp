#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class FeatureExtraction
    {
        // cv::Ptr<T> is like smart pointer in opencv
        cv::Ptr<cv::FeatureDetector> detector;   // detector object declaration 
        cv::Ptr<cv::DescriptorExtractor> descriptor; // descriptor object declaration 
        cv::cuda::GpuMat gpu_descriptors_1, gpu_desriptors_2;
        
        cv::Ptr<cv::DescriptorMatcher> matcher; // feature matcher object declaration 

        //cv::Mat prev_descriptors;
        //prev_descriptors = cv::Mat::zeros(cv::Size(1226,370), CV_8UC1); //descriptors from frame k-1 
        //std::vector<cv::KeyPoint> prev_keyPs; // vector to store keypoints from frame k-1

    public:  
        enum desctiptor_T {sift, orb, surf, brief, akaze, brisk}; 
        int num_features = 5000; 
        cv::Mat descriptors, prev_descriptors, descriptors_1, desriptors_2; // descriptors for left and right frame
        std::vector<cv::KeyPoint> keypoints, keypoints_1, keypoints_2; //vectors of KeyPoints for left and right frame 

        std::vector<cv::DMatch> good_matches; 
        std::vector<std::vector<cv::Point2i>> matched_keypoints; 
        std::vector<cv::KeyPoint> prev_keyPs;

        //FeatureExtraction(); 
        void getFeatures(cv::Mat frame, const desctiptor_T& descriptor_type);
        void getFeatures(cv::cuda::GpuMat frame, const std::string& descriptor_type);
        void matchFeatures(const std::string& matcher_type, const float& low_rt = 0.7f); 

    }; 
   



}