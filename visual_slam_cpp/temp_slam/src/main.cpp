#include <iostream>
#include "../include/StereoDirectSLAM.hpp"
#include "../include/tools.hpp"

int main()
{
    mrVSLAM::StereoDirectSLAM slam("07"); 
    slam.Run(); 
    
    return 0; 
}