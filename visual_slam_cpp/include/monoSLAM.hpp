#pragma once 
#include "system.hpp"


namespace mrVSLAM
{
    class monoSLAM
    {
    private: 
        // Eigen::Matrix3d R; //rotation matrix 
        // Eigen::Vector3d t; //translation vector 
        // Eigen::Matrix3d H, F, K, E;  //homography, fundamental, Intrinsic and essential matrix
        cv::Mat P = cv::Mat(3,4,CV_8UC1), R = cv::Mat(3,3,CV_8UC1); 
        cv::Mat H, F, K, E, t_h = cv::Mat(4,1,CV_8UC1), t = cv::Mat(3,1,CV_8UC1); 

        Eigen::Matrix3d R_e; 
        Eigen::Vector3d t_e; 

        //camera parameters 
        float cx, cy, fx, fy; // fx should be the same as fy 

        void poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points); 

    public:
        int f_counter = 0; 
        std::array<double,6> x_t; // object state x_t = [x,y,z, phi,theta,gamma]
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> poses;
        Eigen::Matrix<double, 3,4, Eigen::RowMajor> Rt; 


        monoSLAM() noexcept {} 
        monoSLAM(const Eigen::Matrix<double,3,4> &projectionMatrix) noexcept; 
        int executeMonoSLAM(const std::string& imgs_path); 
        int executeGPUMonoSLAM(const std::string& imgs_path); 

        enum MatcherType {BruteForce, Flann}; 
        enum ExtractorType {orb, orb_fast, orb_harris, sift, akaze}; 

    }; 


}


