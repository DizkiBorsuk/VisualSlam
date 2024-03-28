#include <iostream>
#include "mrVSLAM/slam.hpp"

int main(int argc, char* argv[]) 
{
    std::string kitti_path = "/home/maciek/dev/projects/VisualSlam/KITTY_dataset/sequences/06"; 
    std::cout << "Hello mrVSLAM \n"; 

    //input argumes handling 
    if(argc>1)
    {
        kitti_path = argv[1]; 
    }
    
    //creat SLAM class instance 
    std::shared_ptr<mrVSLAM::SLAM> slam(new mrVSLAM::SLAM(kitti_path,SLAM_TYPE::STEREO, false)); 

    //run SLAM 
    slam->initSLAM(); 
    slam->outputSlamResult(); 
    slam->outputSlamResult(); 

    return 0;
}
