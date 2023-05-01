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
        static std::string sequence_dir; 
        static std::string gt_poses_dir; 
        static std::string camera_calibration_path;
    public: 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; 

        void readCalibData(const std::string file_path); 
        void getGTposes(const std::string file_path);

    }; 

}