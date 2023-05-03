#include "../include/VO_temp.hpp"

void mrVSLAM::VO::detectFeatures(cv::Mat frame, const std::string& descriptor)
{
    cv::Mat descriptors1, desriptor2; 
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2; 
    if(descriptor == "SIFT")
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create(); 

    }
    else if(descriptor == "ORB")
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();  // cv::Ptr<T> smart pointer in opencv 
        


    }else {
        std::cerr << "Wrong name of the descriptor given" << "\n"; 
    }





}