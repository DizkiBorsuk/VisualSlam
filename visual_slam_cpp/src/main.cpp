
#include "../include/system.hpp"
#include "../include/SLAM.hpp"



int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 
    //cv::cuda::setDevice(0); 
    //--------------------//
    mrVSLAM::SLAM slam(mrVSLAM::SLAM::SlamType::featureStereo, "07"); 
    
    //-------------------//
    return 0; 
}