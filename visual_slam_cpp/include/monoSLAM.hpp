#pragma once 
#include "system.hpp"


namespace mrVSLAM
{
    class monoSLAM
    {
    private: 
        Eigen::Matrix3d R; //rotation matrix 
        Eigen::Vector3d t; //translation vector 
        Eigen::Matrix3d H, F, K, E;  //homography, fundamental, Intrinsic and essential matrix

        void poseEstimationEpiCons(std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::DMatch>); 

    public:
        int f_counter = 0; 

        monoSLAM() noexcept {} 
        monoSLAM(const Eigen::Matrix<double,3,4> projectionMatrix) noexcept; 
        int executeMonoSLAM(const std::string& imgs_path); 
        int executeGPUMonoSLAM(const std::string& imgs_path); 
        void executeStereoSLAM(const std::string& left_camera_path, std::string& right_camera_path, bool gpu = true);

    }; 


}


