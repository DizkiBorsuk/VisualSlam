
#include "../include/system.hpp"
#include "../include/SLAM.hpp"
#include "../include/visualize_data.hpp"



int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 
    //cv::cuda::setDevice(0); 
    //--------------------//
    mrVSLAM::SLAM slam("07"); 
    slam.runMonoSLAM(); 
    slam.showResult(); 
    mrVSLAM::plotPerformance(slam.performance); 
    
    //-------------------//
    return 0; 
}