#include <iostream>
#include "../include/SLAM.hpp"
#include "../include/tools.hpp"

int main()
{
    mrVSLAM::StereoSLAM slam("07"); 
    slam.Run(); 
    
    return 0; 
}