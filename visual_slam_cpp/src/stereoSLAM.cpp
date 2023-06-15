#include "../include/stereoSLAM.hpp"

namespace mrVSLAM
{
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


}