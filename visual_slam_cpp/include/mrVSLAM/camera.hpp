/**
 * @file camera.hpp
 * @author mrostocki 
 * @brief camera pinhole model 
 * @version 0.1
 * @date 2024-03-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once 
#include "mrVSLAM/common_includes.hpp" 

namespace mrVSLAM
{
    class Camera 
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  

        double fx = 0, fy = 0, cx = 0, cy = 0, baseline = 0;  ///< Camera intrinsics
        Sophus::SE3d pose;      ///< extrinsic matrix that describes camera pose in respect to main coordination system (usually identity matrix)
        Sophus::SE3d pose_inv;  ///< inverse of extrinsic matrix 
        Eigen::Matrix3d K;      ///< Intrinsics matrix  
        Eigen::Vector3d t;      ///< translation vector, position of camera in respect to main coordination system
        //! to add distortionCoef; 

    public: 

        /**
         * @brief Construct a new Camera object
         * 
         * @param projection_matrix - Camera projection matrix P = K[R|t]
         * @param size_mult - multiplication factor to scale camera img
         */
        Camera(const Eigen::Matrix<double, 3, 4> &projection_matrix, float size_mult)
        {
            K = projection_matrix.block<3,3>(0,0); //get K from P, works only if R is identity matrix 
            K = K*size_mult; // get correct calibration matrix for scaled img 
            t = projection_matrix.col(3); 
            t = K.inverse()*t;  

            baseline = t.norm(); //normalize translation vector to get distance from coordinate frame center 
            pose = Sophus::SE3d(Sophus::SO3d(), t); //get Matrix that describes position of camera on e.g robot, car
            pose_inv = pose.inverse(); 
            fx = K(0, 0);
            fy = K(1, 1); 
            cx = K(0, 2); 
            cy = K(1, 2); 
        }

        /**
         * @brief Construct a new Camera object
         * 
         * @param fx - focal length in x 
         * @param fy - focal length in y
         * @param cx - img center position in x 
         * @param cy - img center position in y
         * @param baseline - camera distance from coordinate system center 
         * @param pose - 4x4 transformation matrix describing camera pose on robot/car 
         */
        Camera(double fx, double fy, double cx, double cy, double baseline,const Sophus::SE3d &pose, float size_mult)
            : fx(fx), fy(fy), cx(cx), cy(cy), baseline(baseline), pose(pose) 
        {
            pose_inv = pose.inverse();
            K <<fx, 0, cx, 
            0, fy, cy, 
            0, 0, 1; 

            K = K* size_mult; 
        }

        /**
         * @brief get undistorted (rectified) img //! to be implemented
         * @param[in/out] img - source img
         */
        void imgRectification(cv::Mat &img); 

        Sophus::SE3d getPose() const {return pose; } // parameters getters 
        Eigen::Matrix3d getK() const {return K; }

        // coordinate transform: world, camera, pixel
        Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw);
        Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_cw);
        Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);
        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);
        Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_cw, double depth = 1);
        Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw);

    }; 

} //! end of namespace 
