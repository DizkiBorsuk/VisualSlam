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
        const int sadWindow = 9;  
        const int numOfDisparities = sadWindow*16; // number of disparities must be divisible by 16 //https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html#ad985310396dd4d95a003b83811bbc138a21fac9fae6db6a60a940fd24cc61f081
        const int blockSize = 7; // should be in range 3-11
        cv::Ptr<cv::StereoMatcher> stereoMatcher; 

    public: 
        
        // disparity map is map map of diffrence in image location of the same 3d point from 2 diffrent camera angles 
        cv::Mat disparityMap = cv::Mat(370, 1226, CV_16SC1); // not sure if size is correct 
        cv::Mat depthMap = cv::Mat(370, 1226, CV_16SC1);

        StereoDepth(stereoMatcherType, double baseline, double focalLength) noexcept; 

        void calculateDepth(const cv::Mat &leftImg, const cv::Mat &rightImg); 

    }; 

    StereoDepth::StereoDepth(stereoMatcherType matcher, double baseline, double focalLength) noexcept
        : baseline(baseline), f(focalLength)
    {
        switch (matcher)
        {
        case BlockMatching:
            stereoMatcher = cv::StereoBM::create(numOfDisparities, blockSize); 
            break;
        case SGBM:
            stereoMatcher = cv::StereoSGBM::create(0, numOfDisparities, blockSize, 8*12*sadWindow, 32*12*sadWindow, cv::StereoSGBM::MODE_SGBM_3WAY); 
            break;  
        
        default:
            break;
        }
    }
    
    void StereoDepth::calculateDepth(const cv::Mat &leftImg, const cv::Mat &rightImg)
    {
        stereoMatcher->compute(leftImg, rightImg, disparityMap); 
        std::cout << "size of disparity map = " << disparityMap.size()<< "\n"; 
        
    }

}
