
#include "../include/system.hpp"
#include "../include/SLAM.hpp"
#include "../include/readDataset.hpp"
#include "../include/visualize_data.hpp"



int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 

    ///// ------ Read Calibration data and Ground Truth Poses ---- //////
    mrVSLAM::KITTI_Dataset kitti("06"); 
    kitti.readCalibData(); 
    kitti.showPmatricies(); 

    kitti.getGTposes(); 
    std::cout << std::setprecision(1) << std::fixed << kitti.ground_truth_poses[0] <<  "\n"; 
    std::cout << "\n" << "-------------"<<"\n"; 

    //--------------------//
    /* Main algorithm */
    
    mrVSLAM::SLAM slam; 
    slam.executeMonoSLAM(kitti.left_imgs_path); 




    //-------------------//
    

    /* Results */

    mrVSLAM::plotPoses(kitti.ground_truth_poses, slam.f_counter); 


    return 0; 
}