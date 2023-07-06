
#include "../include/system.hpp"
#include "../include/monoSLAM.hpp"
#include "../include/readDataset.hpp"
#include "../include/visualize_data.hpp"
#include "../include/camera.hpp"



int counter = 0; 

int main(int argc, char** argv)
{
    std::cout << "Visual Slam start \n";  
    cv::cuda::printCudaDeviceInfo(0); 
    //cv::cuda::setDevice(0); 



    ///// ------ Read Calibration data and Ground Truth Poses ---- //////
    mrVSLAM::KITTI_Dataset dataset("07"); 
    dataset.readCalibData(); 
    dataset.showPmatricies(); 

    dataset.getGTposes(); 
    std::cout << std::setprecision(1) << std::fixed << dataset.ground_truth_poses[0] <<  "\n"; 
    std::cout << "\n" << "-------------"<<"\n"; 

    std::shared_ptr<mrVSLAM::Camera> camera(new mrVSLAM::Camera(dataset.P0)); 
    std::shared_ptr<mrVSLAM::Camera> camera_right(new mrVSLAM::Camera(dataset.P0));  


    //--------------------//
    /* Main algorithm */
    
    mrVSLAM::monoSLAM slam; 
    slam.executeMonoSLAM(dataset.left_imgs_path); 


    //-------------------//
    
    //cv::cuda::resetDevice(); 
    /* Results */

    mrVSLAM::plotPoses(dataset.ground_truth_poses, slam.poses, slam.f_counter); 


    return 0; 
}