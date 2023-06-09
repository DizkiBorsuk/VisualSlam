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
        float cx, cy; 
        void poseEstimationEpiCons(std::vector<std::vector<cv::Point2f>> &matched_points, std::vector<cv::DMatch> &matches); 

    public:
        int f_counter = 0; 
        std::array<double,6> x_t; // object state x_t = [x,y,z, phi,theta,gamma]

        monoSLAM() noexcept {} 
        monoSLAM(const Eigen::Matrix<double,3,4> projectionMatrix) noexcept; 
        int executeMonoSLAM(const std::string& imgs_path); 
        int executeGPUMonoSLAM(const std::string& imgs_path); 

        enum MatcherType {BruteForce, Flann}; 
        enum ExtractorType {orb, orb_fast, orb_harris, sift, akaze}; 

    }; 


}


