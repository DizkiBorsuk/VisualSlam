#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    enum stereoMatcherType {BlockMatching, SGBM}; 
    // class or just function, that is a question
    class StereoDepth
    {
    private:
        double baseline = 0 ;
        double f = 0;  
        double M = 0; // f*baseline 
        static constexpr int minDisparity = 0; 
        static constexpr int sadWindow = 9;  
        static constexpr int numOfDisparities = sadWindow*16; // number of disparities must be divisible by 16 //https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html#ad985310396dd4d95a003b83811bbc138a21fac9fae6db6a60a940fd24cc61f081
        static constexpr int blockSize = 15; // should be in range 3-11
        cv::Ptr<cv::StereoMatcher> stereoMatcher; 

        cv::Mat disparity16 = cv::Mat(370, 1226, CV_16S);

    public: 
        
        // disparity map is map map of diffrence in image location of the same 3d point from 2 diffrent camera angles 
        cv::Mat disparityMap = cv::Mat(370, 1226, CV_32FC1); 
        cv::Mat depthMap = cv::Mat(370, 1226, CV_32FC1);

        StereoDepth(stereoMatcherType, double baseline, double focalLength) noexcept; 

        void calculateDepth(const cv::Mat &leftImg, const cv::Mat &rightImg); 

    }; 

    StereoDepth::StereoDepth(stereoMatcherType matcher, double baseline, double focalLength) noexcept
        : baseline(baseline), f(focalLength)
    {
        M = baseline*f; 

        switch (matcher)
        {
        case BlockMatching:
            stereoMatcher = cv::StereoBM::create(numOfDisparities, blockSize); 
            break;
        case SGBM:
            stereoMatcher = cv::StereoSGBM::create(0, numOfDisparities, blockSize, 24*sadWindow*sadWindow, 96*sadWindow*sadWindow, cv::StereoSGBM::MODE_SGBM_3WAY); 
            break;  
        
        default:
            break;
        }
    }
    
    void StereoDepth::calculateDepth(const cv::Mat &leftImg, const cv::Mat &rightImg)
    {
        stereoMatcher->compute(leftImg, rightImg, disparity16); 
         
        disparity16.convertTo(disparityMap, CV_32F, 1.0); 
        disparityMap = (disparityMap/16.0f - (float)minDisparity)/((float)numOfDisparities);

        depthMap = M / disparityMap;  
        std::cout << "depthMap = \n" << disparityMap; 
    }

}
