#pragma once 
#include "system.hpp"

namespace mrVSLAM{
    class KITTI_Dataset
    {
    private: 
 
    public: 
        std::string gt_poses_path; 
        std::string camera_calibration_path; 
        std::string left_imgs_path; 
        std::string right_imgs_path;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; //Projection matrix of left grayscale camera  
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1; //Projection matrix of right grayscale camera
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2; //Projection matrix of left rgb camera - unused, I don't need rgb images 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; //poses are reprensented as a 3x4 transformation matrix:  3x3 - rotation matrix + 3x1 translation vector

        KITTI_Dataset(const std::string& sequence); //initialize paths to camera images, ground truth poses and calibration files
        void readCalibData(); //get camera projection matrixies from calibration file 
        void getGTposes(); //get set of ground truth poses 
        void showPmatricies(); 

    }; 

    class EuRoCMAV_Dataset
    {

    EuRoCMAV_Dataset(); 
    
    }; 
    
}