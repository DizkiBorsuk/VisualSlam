#include "../include/stereoSLAM.hpp"
#include <cmath>

namespace mrVSLAM
{
    StereoSLAM::StereoSLAM(const Eigen::Matrix<double,3,4> &P_left, const Eigen::Matrix<double,3,4> &P_right, const int &stereoMatcherType)
    {
        if(stereoMatcherType == 1)
        {
            stereoMatcher = cv::StereoBM::create(numOfDisparities, blockSize); 
        }
        else if(stereoMatcherType == 2)
        {
            stereoMatcher == cv::StereoSGBM::create(0, numOfDisparities,blockSize, pow(16*sad_window,2), pow(96*sad_window,2), 0, 0, 0, 0, 0, cv::StereoSGBM::MODE_SGBM_3WAY); 
        }
        else
        {

        }
    }

    int StereoSLAM::executeStereoSLAM(const std::string& left_imgs_path, const std::string& right_imgs_path)
    {

        cv::Mat left_frame(1226,370, CV_8UC1), right_frame(1226,370, CV_8UC1); 
        int start, end, framesPerSecond; 

        cv::VideoCapture left_sequence; 
        cv::VideoCapture right_sequence; 
        left_sequence.open(left_imgs_path, cv::CAP_IMAGES);
        right_sequence.open(right_imgs_path, cv::CAP_IMAGES);


        cv::namedWindow("Camera Img", cv::WINDOW_AUTOSIZE); 

        if (!left_sequence.isOpened() || !right_sequence.isOpened())
        {
        std::cerr << "Failed to open Image Sequence!\n"; 
        return -1;
        }

        while(true)
        {
            left_sequence.read(left_frame);
            right_sequence.read(right_frame);
            
            if(left_frame.empty() || right_frame.empty())
            {
                std::cout << "End of sequance \n"; 
                break;
            }

            
        }

    }

    void computeDisparityMap(const cv::Mat &left_img, const cv::Mat &right_img); 
    {


        if(stereoMatcherType == 1)
        {

        }
        

    }


}