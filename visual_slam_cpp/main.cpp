#include "mrVSLAM/slam.hpp"


int main(int argc, char* argv[]) 
{
    std::string kitti_path = "/home/maciek/dev/projects_cpp/VisualSlam/KITTY_dataset/sequences/06"; 
    bool use_loop_closer = false; 


    fmt::print("Hello mrVSLAM \n"); 

    //input argumes handling 
    if(argc>1)
    {
        fmt::print(fg(fmt::color::green), "using terminal input arguments \n"); 
        kitti_path = argv[1]; 
        use_loop_closer = argv[2]; 

    } else { fmt::print(fg(fmt::color::red),"no input arguments, using default inputs \n"); }
    
    //creat SLAM class instance 
    std::shared_ptr<mrVSLAM::SLAM> slam = std::make_shared<mrVSLAM::SLAM>(kitti_path,SLAM_TYPE::STEREO, use_loop_closer); 

    //run SLAM 
    slam->setSlamParameters(DetectorType::GFTT,150, 1.0f); 
    slam->initSLAM(); 
    slam->runSLAM(); 
    slam->outputSlamResult(); 

    return 0;
}
