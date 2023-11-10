#include "myslam/slam.hpp"

int main() {
    
    std::cout << "Hello mrVSLAM \n"; 
    std::string kitti_path = "/home/maciek/dev/projects/VisualSlam/KITTY_dataset/sequences/07"; 

    std::shared_ptr<myslam::StereoSLAM> slam(new myslam::StereoSLAM(kitti_path, 0.5));
    slam->Init();
    slam->Run();

    slam->output(); 

    return 0;
}
