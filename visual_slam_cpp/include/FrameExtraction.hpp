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


    inline void extraxtFeatures(const cv::Mat &img, const ExtractorType, const int numberOfFeatures, std::vector<cv::KeyPoint> &outKeypoint, cv::Mat &outDescriptors) noexcept; 
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures) noexcept; 


    class Frame
    {
    public: 
        int frameId; 
        cv::Matx<double, 3, 3> K, invK;
        cv::Matx<double, 4, 4> pose;  

        std::vector<cv::KeyPoint> frameFeaturePoints;  
        cv::Mat frameDescriptors = cv::Mat(32, 600, CV_8UC1);
        
        Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures) noexcept;
    private: 

    };

    class FrameMatcher
    {
    public: 
        std::vector<cv::DMatch> goodMatches; 
        std::vector<std::vector<cv::Point2f>> matchedKeypoints; 

        FrameMatcher() noexcept; 

        void matchFramesFlann(const Frame &frame1, const Frame &frame2, const float& low_rt = 0.7f) noexcept; 
        void matchFramesBF(const Frame &frame1, const Frame &frame2, const float& low_rt = 0.7f) noexcept;

    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher; 

    };





}