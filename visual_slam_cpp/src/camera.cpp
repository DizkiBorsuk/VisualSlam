#include "../include/camera.hpp"

namespace mrVSLAM
{
    void Camera::setCamera(const Eigen::Matrix<double,3,4> &projectionMatrix) noexcept
    {
        // get camera calibration parameters 
        cx = projectionMatrix.coeff(0,2); 
        cy = projectionMatrix.coeff(1,2); 
        fx = projectionMatrix.coeff(0,0); 
        fy = projectionMatrix.coeff(1,1); 
        principialPoint = cv::Point(cx, cy); 

        // projection matrixies in cv::mat format 
        cv::Mat P(cv::Size(3,4), CV_32FC1);
        cv::Mat th; //homogenous translation vector
        cv::Mat temp_t; 

        cv::eigen2cv(projectionMatrix, P); // convert camera eigen matrix to cv::Mat format 

        //decompose both projection matrixies 
        cv::decomposeProjectionMatrix(P, K, R, th); 
        //convert th and thr to euclidian space 
        cv::convertPointsFromHomogeneous(th.reshape(4,1), temp_t); //https://answers.opencv.org/question/176008/opencv-error-assertion-failed-_dstfixedtype-in-cvconvertpointshomogeneous/

        t = cv::Matx31d((double*)temp_t.ptr()); // convert cv::Mat to cv::Matx , the most idiotic thing ever 
        cv::cv2eigen(K, K_eigen); 
        Rt << R(0,0), R(0,1), R(0,2), t(0,0), 
              R(1,0), R(1,1), R(1,2), t(1,0), 
              R(2,0), R(2,1), R(2,2), t(2,0), 
              0, 0, 0, 1; 
    }


    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &point_in_world, const Eigen::Matrix4d Tcw)
    {
        /* 
        Tcw - Transformation matrix from world to camera, transformations are read from right to left https://github.com/raulmur/ORB_SLAM2/issues/226
        */
        return (Rt*Tcw).block<3,3>(0,0)*point_in_world; 
    } 

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &point_in_camera, const Eigen::Matrix4d Twc)
    {
        /*
        Twc = Tcw.inverse()
        */
        Eigen::MatrixXd temp_mt = Twc*Rt.inverse(); 
        return temp_mt.block(0,0,3,3)*point_in_camera; 
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &point_in_camera)
    {
        return Eigen::Vector2d(fx*point_in_camera(0,0) / point_in_camera(2,0) + cx, 
                               fy*point_in_camera(1,0)/ point_in_camera(2,0) + cy);  // f*x/z + cx ; f*y/z + cy //*camera exuations
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &pixel_positon, double alfa)
    {
        return Eigen::Vector3d( (pixel_positon(0,0) - cx)*alfa/fx, 
                                (pixel_positon(1,0) - cy)*alfa/fy, 
                                alfa); 
    }
    // Eigen::Vector3d Camera::pixel2camera(const cv::KeyPoint &pixel_positon, double alfa)
    // {
    //     return Eigen::Vector3d( (pixel_positon.pt.x - cx)*alfa/fx, 
    //                             (pixel_positon.pt.y - cy)*alfa/fy, 
    //                             alfa);
    // }       

    double getStereoBaseline(const cv::Matx<double, 3, 1> &t1, const cv::Matx<double, 3, 1> &t2) noexcept
    {
        return abs(t1(0) - t2(0)); 
    }
}