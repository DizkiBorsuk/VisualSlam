#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    enum ExtractorType {OrbHarris, OrbFast, ORB, OrbGptt, SIFT, AKAZE}; 
    enum MatcherType {BruteForce, Flann}; 

    struct feature
    {
        std::vector<cv::KeyPoint> frameKeypoints; 
        cv::Mat descriptors; 
    };

    struct gpuFeature
    {
        cv::cuda::GpuMat gpuKeypoints; 
        cv::cuda::GpuMat gpuDescriptors;  
    };


    feature extraxtFeatures(const cv::Mat &img, const ExtractorType, const int numberOfFeatures); 
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures); 


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




    class Frame
    {
    public: 
        int frameId; 

        Eigen::Matrix3d K_eigen, invK_eigen; 
        Eigen::Matrix4d pose_eigen; 

        cv::Matx<double, 3, 3> K, invK;
        cv::Matx<double, 4, 4> pose;  

        std::vector<cv::KeyPoint> frameFeaturePoints;  
        cv::Mat frameDescriptors; 
        
        Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures);
    private: 

    };
}