#include "../include/FeatureExtraction.hpp"



void mrVSLAM::FeatureExtraction::getFeatures(cv::Mat frame, const desctiptor_T& descriptor_type)
{
    switch(descriptor_type)
    {
        case sift: 
            detector = cv::SIFT::create(num_features);  
            descriptor = cv::SIFT::create(num_features); 
            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        case orb: 
            detector = cv::ORB::create(num_features);  
            descriptor = cv::ORB::create(num_features); 
            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        case akaze: 
            detector = cv::AKAZE::create(); 
            descriptor = cv::AKAZE::create(); 

            detector->detect(frame, keypoints);
            descriptor->compute(frame,keypoints,descriptors); 
            break; 
        default: 
            std::cerr << "Wrong name of the descriptor given" << "\n"; 
            break;  

    }
}

void mrVSLAM::FeatureExtraction::matchFeatures()
{

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
    // matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE); 
    // matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); 

    std::vector<cv::DMatch> good_matches; 
    cv::Mat prev_descriptors;  //descriptors from frame k-1 
    std::vector<cv::KeyPoint> prev_keyP; // vector to store keypoints from frame k-1
    std::vector<std::vector<cv::DMatch>> matches;

    const float ration_treshold = 0.75f; 

    if(!prev_descriptors.empty())
    {
        matcher->knnMatch(descriptors_1, prev_descriptors, matches,2); 
        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches[i][0].distance < ration_treshold * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);

            }
        }
    }

    if(good_matches.size() > 0)
    {


    }

    


    prev_descriptors = descriptors_1; 
}





// void mrVSLAM::FeatureExtraction::getFeatures(cv::cuda::GpuMat frame, const std::string& descriptor_type)
// {
//     if(descriptor_type == "SURF")
//     {


//     }
//     else if(descriptor_type == "ORB")
//     {
//         cv::Ptr<cv::cuda::ORB> gpu_detector = cv::cuda::ORB::create(num_features,1.200000048F, 8, 31, 0, 2, 0, 31, 20, true);  
//         //gpu_ORB->detectAndComputeAsync(frame, cv::noArray(), gpu_keypoints_1, gpu_descriptors_1, false); 
//         //gpu_detector->detectAndCompute(frame, cv::noArray(), keypoints_1, gpu_descriptors_1, false); 
        
//         gpu_detector->detect(frame, keypoints_1);
//         std::cout << "Number of features: " << keypoints_1.size() << "\n"; 
//         //gpu_ORB->compute(frame, gpu_keypoints_1, descriptors_1_gpu); 
//         //gpu_ORB->convert(gpu_keypoints_1, keypoints_1); 


//     }else {
//         std::cerr << "Wrong name of the descriptor given" << "\n"; 
//     }

//}
