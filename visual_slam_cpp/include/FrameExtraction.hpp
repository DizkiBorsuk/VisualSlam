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
    gpuFeature extraxtGpuFeatures( const cv::cuda::GpuMat &img, const int numberOfFeatures) noexcept; 

    class FeatureExtractor
    {
    public: 
        FeatureExtractor(const ExtractorType &extractor, const int &numberOfFeatures) noexcept;
        void extractFeatures(const cv::Mat &img, std::vector<cv::KeyPoint> &outKeypoint, cv::Mat &outDescriptors) noexcept; 
        //void (FeatureExtractor::*extraxtFeaturesPointer)(const cv::Mat &img, std::vector<cv::KeyPoint> &outKeypoint, cv::Mat &outDescriptors) = &FeatureExtractor::extraxtFeatures; 

    private: 
        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor>  descriptor; 
    };
    

    class Frame
    {
    public: 
        int frameId; // id of a frame
        cv::Matx<double, 3, 3> K; //invK;
        cv::Matx<double, 4, 4> pose;  
        bool isKeyFrame = false; 
        std::shared_ptr<Frame> framePtr; 

        std::vector<cv::Point2f> points; //points after matching 
        std::vector<cv::KeyPoint> frameFeaturePoints; // keypoints from extractor  
        cv::Mat frameDescriptors = cv::Mat(32, 800, CV_8UC1); // descriptors
    
        Frame(const cv::Mat &image, cv::Matx33d &cameraMatrix, const int frameId, const int numOfFeatures, FeatureExtractor *) noexcept;    
    };

    class FrameMatcher
    {
    public: 
        //std::vector<cv::DMatch> goodMatches; 
        std::vector<std::array<cv::Point2f, 2>> matchedKeypoints; 

        FrameMatcher(MatcherType) noexcept; 
        void matchFrames(Frame &frame1, Frame &frame2, const float &low_ratio = 0.7f) noexcept; 
        

    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher; 

        enum _descriptorType {float32, uchar8};
        _descriptorType descT; 
        cv::Mat descriptors1; 
        cv::Mat descriptors2; 
    };

}