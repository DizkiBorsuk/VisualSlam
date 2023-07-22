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
        const int sad_window = 9;  
        const int numOfDisparities = sad_window*16; 
        const int blockSize = 11; 
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
            stereoMatcher = cv::StereoBM::create(); 
            break;
        case SGBM:
            stereoMatcher = cv::StereoSGBM::create(); 
            break;  
        
        default:
            break;
        }
    }
   

}
