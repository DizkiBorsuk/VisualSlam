#include "../include/camera.hpp"


namespace mrVSLAM
{
    Camera::Camera(const Eigen::Matrix<double,3,4> &projectionMatrix)
    {
        // get camera calibration parameters 
        cx = projectionMatrix.coeff(0,2); 
        cy = projectionMatrix.coeff(1,2); 
        fx = projectionMatrix.coeff(0,0); 
        fy = projectionMatrix.coeff(1,1); 
        principialPoint = {cx,cy}; 

        // projection matrixies in cv::mat format 
        cv::Mat P(cv::Size(3,4), CV_32FC1);
        //
        cv::Mat th(cv::Size(4,1), CV_32FC1); //homogenous translation vector

        cv::eigen2cv(projectionMatrix, P); // convert camera eigen matrix to cv::Mat format 

        //decompose both projection matrixies 
        cv::decomposeProjectionMatrix(P, K_mat, R_mat, th); 

        //convert th and thr to euclidian space 
        cv::convertPointsFromHomogeneous(th, t_mat); 

        std::cout << "cx = " << cx <<"\n cy = " << cy <<"\n f = " <<fx <<"\n"; 
        std::cout << "t1 = " << t_mat << "R = " << R_mat << "\n K = " << K_mat << "\n"; 
    }
}