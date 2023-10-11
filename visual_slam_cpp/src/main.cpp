#include <iostream>
#include "../include/SLAM.hpp"
#include "../include/frame.hpp"
#include "../include/map.hpp"
#include "../include/tools.hpp"

unsigned int mrVSLAM::Frame::keyframe_counter = 0; // well, that's stupid but i don't have better idea 
unsigned int mrVSLAM::MapPoint::mappoint_counter = 0; 

int main()
{
    std::cout << "Hello mrVSLAM \n"; 
    mrVSLAM::StereoSLAM slam("07"); 

    slam.Run(); 

    mrVSLAM::plotPoses(slam.trajectory, slam.trajectory.size()); 
    
    return 0; 
}