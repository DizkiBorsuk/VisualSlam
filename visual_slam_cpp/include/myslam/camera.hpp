#pragma once
#include "myslam/common_include.hpp"

namespace myslam {

// Pinhole stereo camera model
    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
            baseline_ = 0;  // Camera intrinsics
        Sophus::SE3d pose_;             // extrinsic, from stereo camera to single camera
        Sophus::SE3d pose_inv_;         // inverse of extrinsics
        Eigen::Matrix3d K; 
        Eigen::Vector3d t; 

        Camera(const Eigen::Matrix<double,3,4> &projection_matrix, float size_mult) 
        { 
            K = projection_matrix.block<3,3>(0,0); 
            K = K*size_mult; 
            t = projection_matrix.col(3); 
            t = K.inverse() * t; 

            baseline_ = t.norm(); 
            pose_ = Sophus::SE3d(Sophus::SO3d(), t); 
            fx_ = K(0, 0);
            fy_ = K(1, 1); 
            cx_ = K(0, 2); 
            cy_ = K(1, 2); 
        }

        Camera(double fx, double fy, double cx, double cy, double baseline,
            const Sophus::SE3d &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
        }

        Sophus::SE3d pose() const { return pose_; }

        // return intrinsic matrix
        Eigen::Matrix3d getK() const {
            Eigen::Matrix3d k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return k;
        }

        // coordinate transform: world, camera, pixel
        Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

        Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w);

        Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);

        Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth = 1);

        Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);
    };

} 

