#include "../include/FeatureExtraction.hpp""





void mrVSLAM::FeatureExtraction::getFeatures(cv::Mat frame, const std::string& descriptor_type)
{
     
    if(descriptor_type == "SIFT")
    {
        cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create(); 

    }
    else if(descriptor_type == "ORB")
    {
        detector = cv::ORB::create();  // cv::Ptr<T> smart pointer in opencv 
        descriptor = cv::ORB::create(); 
        detector->detect(frame, keypoints_1);
        descriptor->compute(frame,keypoints_1,descriptors_1); 
        
    }else {
        std::cerr << "Wrong name of the descriptor given" << "\n"; 
    }
}

void mrVSLAM::FeatureExtraction::getFeatures(cv::cuda::GpuMat frame, const std::string& descriptor_type)
{


}
