#include "myslam/slam.hpp"

int main() {
    
    std::cout << "Hello mrVSLAM \n"; 
    std::string kitti_path = "/home/maciek/dev/projects/VisualSlam/KITTY_dataset/sequences/06"; 
    //std::string kitti_path = "/home/maciek/dev/projects/VisualSlam/KITTY_dataset/sequences/07_color"; 

    std::shared_ptr<myslam::SLAM> slam(new myslam::SLAM(kitti_path, myslam::slamType::stereo_opf, true, 1));
    slam->Init();
    slam->Run();

    slam->output(); 

    return 0;
}
