#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class FeatureExtraction
    {

        cv::Ptr<cv::FeatureDetector> detector; 
        cv::Ptr<cv::DescriptorExtractor> descriptor; 
    public: 
        cv::Mat descriptors_1, desriptors_2; // descriptors for left and right frame
        cv::cuda::GpuMat descriptors_1_gpu, desriptors_2_gpu; //descriptrs for gpu implementation
        std::vector<cv::KeyPoint> keypoints_1, keypoints_2; //vectors of KeyPoints for left and right frame 

        void getFeatures(cv::Mat frame, const std::string& descriptor_type);
        void getFeatures(cv::cuda::GpuMat frame, const std::string& descriptor_type);

    }; 
   



}