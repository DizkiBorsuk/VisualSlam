#pragma once 
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "Eigen/Dense"

namespace mrVSLAM{
    class KITTI_Dataset
    {
    private: 
 
    public: 
        std::string gt_poses_path; 
        std::string camera_calibration_path;
        std::string left_imgs_path; 
        std::string right_imgs_path;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; 

        KITTI_Dataset(std::string sequence); 
        void readCalibData(std::string file_path); 
        void getGTposes(std::string file_path);

    }; 

}