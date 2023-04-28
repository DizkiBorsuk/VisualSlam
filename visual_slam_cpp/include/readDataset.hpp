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
        std::string sequence_dir; 
        std::string gt_poses_dir; 
        std::string camera_calibration_path;
    public: 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;

        void readCalibData(std::string file_path); 

    }; 

}