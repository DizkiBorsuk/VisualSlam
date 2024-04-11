/**
 * @file read_dataset.hpp
 * @author your name (you@domain.com)
 * @brief reading kitti dataset stuff 
 * @version 0.1
 * @date 2024-03-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once 
#include "mrVSLAM/common_includes.hpp" 

namespace mrVSLAM
{
    /**
     * @brief class for reading in kitti dataset https://www.cvlibs.net/datasets/kitti/
     * 
     */
    class KITTI_Dataset
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; //Projection matrix of left grayscale camera  
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1; //Projection matrix of right grayscale camera
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2; //Projection matrix of left rgb camera - unused, I don't need rgb images 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; //poses are reprensented as a 3x4 transformation matrix:  3x3 - rotation matrix + 3x1 translation vector

        KITTI_Dataset(const std::string dataset_path); 
        void readCalibData(); //get camera projection matrixies from calibration file 
        void getGTposes(); //get set of ground truth poses 
        void showPmatricies() const; 
        unsigned int getCurrentSequence(); 

    private: 
        std::string path_to_dataset; 
        std::string camera_calibration_path; 
        std::string gt_poses_path; 
    }; 

} //! end of namespace 