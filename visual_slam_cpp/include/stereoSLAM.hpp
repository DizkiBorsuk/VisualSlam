#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class StereoSLAM
    {
    private:
        const int sad_window = 6;  
        const int numOfDisparities = sad_window*16; 
        const int blockSize = 11; 
        cv::Ptr<cv::StereoMatcher> stereoMatcher; 

    public: 
        int f_counter = 0;  
        cv::Mat disparityMap; 

        StereoSLAM() noexcept {}; 
        StereoSLAM(const Eigen::Matrix<double,3,4> &P_left, const Eigen::Matrix<double,3,4> &P_right, const int &stereoMatcherType); 
        int executeStereoSLAM(const std::string& left_imgs_path, const std::string& right_imgs_path);

        cv::Mat computeDisparityMap(const cv::Mat &left_img, const cv::Mat &right_img); 


    }; 


}
