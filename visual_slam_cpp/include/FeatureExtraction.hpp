#pragma once 
#include "system.hpp"

namespace mrVSLAM
{
    class FeatureExtraction
    {
        int num_features = 500; 
        cv::Ptr<cv::FeatureDetector> detector;  // cv::Ptr<T> is like smart pointer in opencv
        cv::Ptr<cv::DescriptorExtractor> descriptor; 
        cv::cuda::GpuMat gpu_keypoints_1, gpu_keypoints_2; 
        cv::cuda::GpuMat gpu_descriptors_1, gpu_desriptors_2;

    public: 
        cv::Mat descriptors_1, desriptors_2; // descriptors for left and right frame
        std::vector<cv::KeyPoint> keypoints_1, keypoints_2; //vectors of KeyPoints for left and right frame 

        void getFeatures(cv::Mat frame, const std::string& descriptor_type);
        void getFeatures(cv::cuda::GpuMat frame, const std::string& descriptor_type);

    }; 
   



}