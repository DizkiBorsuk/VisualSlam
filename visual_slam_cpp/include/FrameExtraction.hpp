#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    enum ExtractorType {OrbHarris, OrbFast, ORB, OrbGptt, SIFT, AKAZE}; 
    enum MatcherType {BruteForce, Flann}; 

    class FeatureExtraction
    {
    public:  
        std::vector<cv::KeyPoint> frame_keypoints; 
        cv::Mat descriptors; 
        cv::cuda::GpuMat gpuDescriptors, gpuKeypoints; 

        //default constructor and non-dafault constructor 
        FeatureExtraction() noexcept {} //default constructor 
        FeatureExtraction(const ExtractorType, const bool GPU, const int numberOfFeatures) noexcept;

        void getFeatures(const cv::Mat frame) noexcept; 
        void getGPUFeatures(const cv::cuda::GpuMat frame) noexcept;

    private:
        // CPU detector and descriptor objects declaration 
        cv::Ptr<cv::FeatureDetector> detector;   // detector object declaration  
        cv::Ptr<cv::DescriptorExtractor> descriptor; 

        // GPU detector and descriptor 
        cv::Ptr<cv::cuda::ORB> gpuOrbExtractor;
        cv::Ptr<cv::cuda::FastFeatureDetector> gpuFastDetector; 

    }; 

    class FrameMatcher
    {
    public: 
        std::vector<cv::DMatch> goodMatches; 
        std::vector<std::vector<cv::Point2f>> matchedKeypoints; 

        FrameMatcher() noexcept {} 
        FrameMatcher(const Frame &frame1, const Frame &frame2, MatcherType, const float low_rt = 0.7f) noexcept; 


        void matchFeaturesFlann(const float& low_rt = 0.7f) noexcept; 
        void matchFeaturesBF(const float& low_rt = 0.7f) noexcept;

    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher; 

    };


    class Frame
    {
    public: 
        Eigen::Matrix3d K_eigen, invK_eigen; 
        Eigen::Matrix4d pose_eigen; 
        std::vector<cv::KeyPoint> frameFeaturePoints;  
        cv::Mat descriptors; 
        
        int frameId; 

        Frame(const cv::Mat &image, cv::Mat &cameraMatrix, );
    private: 

    };
}