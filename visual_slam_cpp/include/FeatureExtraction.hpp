#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class FeatureExtraction
    {
    private:
        // cv::Ptr<T> is like smart pointer in opencv
        cv::Ptr<cv::FeatureDetector> detector;   // detector object declaration  
        cv::Ptr<cv::DescriptorExtractor> descriptor; // descriptor object declaration 
        
        cv::Ptr<cv::cuda::ORB> gpu_orb_extractor;
        cv::Ptr<cv::cuda::FastFeatureDetector> gpu_fast_detector; 

        cv::cuda::GpuMat gpu_descriptors, gpu_prev_descriptors, gpu_keypoints, prev_gpu_keyPs;
        cv::Mat descriptors, prev_descriptors;


        cv::Ptr<cv::DescriptorMatcher> matcher; // feature matcher object declaration 
        std::vector<cv::KeyPoint> prev_keyPs;
        
        cv::Ptr<cv::cuda::DescriptorMatcher> gpu_matcher; 


        //
        typedef std::vector<uint32_t> DescType;
        void computeORB(const cv::Mat &frame, std::vector<cv::KeyPoint> &key_points, std::vector<DescType> &descriptors); 


    public:  
        std::vector<cv::KeyPoint> keypoints; 
        std::vector<cv::DMatch> good_matches; 
        std::vector<std::vector<cv::Point2i>> matched_keypoints; 


        FeatureExtraction(const std::string &desType, const bool GPU); 
        //FeatureExtraction(const std::string &desType = "orb", const bool GPU, int num_of_features = 500);
        void getFeatures(const cv::Mat &frame) noexcept; 
        void getFeatures(const cv::cuda::GpuMat &frame) noexcept;
        void matchFeaturesFlann(const float& low_rt = 0.7f) noexcept; 
        void matchFeaturesBF(const float& low_rt = 0.7f) noexcept;
        void matchGPUFeaturesBF(const float& low_rt = 0.7f) noexcept;


    }; 
   



}