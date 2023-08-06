#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    enum ExtractorType {OrbHarris, OrbFast, ORB, OrbGptt, SIFT, AKAZE}; 
    enum MatcherType {BruteForce, Flann}; 

    struct gpuFeature
    {
        cv::cuda::GpuMat gpuKeypoints; 
        cv::cuda::GpuMat gpuDescriptors;  
    };


    inline void extraxtFeatures(const cv::Mat &img, const ExtractorType, const int numberOfFeatures, 
                                std::vector<cv::KeyPoint> &outKeypoint, cv::Mat &outDescriptors) noexcept; 
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures) noexcept; 


    class Frame
    {
    public: 
        int frameId; 
        cv::Matx<double, 3, 3> K, invK;
        cv::Matx<double, 4, 4> pose;  

        std::vector<cv::KeyPoint> frameFeaturePoints;  
        cv::Mat frameDescriptors = cv::Mat(32, 800, CV_8UC1);
        
        Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures) noexcept;
    private: 

    };

    class FrameMatcher
    {
    public: 
        std::vector<cv::DMatch> goodMatches; 
        std::vector<std::array<cv::Point2f, 2>> matchedKeypoints; 

        FrameMatcher(MatcherType) noexcept; 
        void matchFrames(const Frame &frame1, const Frame &frame2, const float& low_ratio = 0.7f) noexcept; 
        

    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher; 
        cv::Point2f keypoint1, keypoint2; 
        std::array<cv::Point2f, 2> pointPair; 

        enum _descriptorType {float32, uchar8};
        _descriptorType descT; 
        cv::Mat descriptors1; 
        cv::Mat descriptors2; 
    };


}