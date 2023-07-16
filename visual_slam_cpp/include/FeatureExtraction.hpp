#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    enum ExtractorType {featureMono, featureStereo, direct}; 
    enum MatcherType {}; 


    class FeatureExtraction
    {

    public:  
        std::vector<cv::KeyPoint>  frame_keypoints; 
        cv::Mat descriptors; 


        FeatureExtraction() noexcept {} //default constructor 
        FeatureExtraction(const ExtractorType, const bool GPU, const int numberOfFeatures) noexcept;

        void getFeatures(const cv::Mat frame) noexcept; 
        void getGPUFeatures(const cv::cuda::GpuMat frame) noexcept;

        //void matchGPUFeaturesBF(const float& low_rt = 0.7f) noexcept;
    private:
        // CPU detector and descriptor objects declaration 
        cv::Ptr<cv::FeatureDetector> detector;   // detector object declaration  
        cv::Ptr<cv::DescriptorExtractor> descriptor; 

        cv::Ptr<cv::cuda::ORB> gpuOrbExtractor;
        cv::Ptr<cv::cuda::FastFeatureDetector> gpuFastDetector; 

    }; 

    class FrameMatcher
    {
    public: 
        std::vector<cv::DMatch> good_matches; 
        std::vector<std::vector<cv::Point2f>> matched_keypoints; 
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

        Frame(const cv::Mat &img, )
    private: 

    };
}