#pragma once 
#include "myslam/common_include.hpp"

namespace myslam
{
    class KITTI_Dataset
    {
    private: 
 
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        std::string dataset_path; 

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; //Projection matrix of left grayscale camera  
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1; //Projection matrix of right grayscale camera
        // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2; //Projection matrix of left rgb camera - unused, I don't need rgb images 
        // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; //poses are reprensented as a 3x4 transformation matrix:  3x3 - rotation matrix + 3x1 translation vector

        KITTI_Dataset(const std::string dataset_path); 
        void readCalibData(); //get camera projection matrixies from calibration file 
        void getGTposes(); //get set of ground truth poses 
        void showPmatricies() const; 

        private: 
            std::string path_to_dataset; 
            std::string camera_calibration_path; 
            std::string gt_poses_path; 
    }; 

    class EuRoCMAV_Dataset
    {

        EuRoCMAV_Dataset(); 
    
    }; 
    
}