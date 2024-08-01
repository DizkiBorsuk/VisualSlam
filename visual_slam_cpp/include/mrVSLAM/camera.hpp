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
        std::vector<double> dist_coeffs;
        Eigen::Matrix<double, 3,4> P_matrix; 
        cv::Size img_size; 

    public:

        /**
         * @brief Construct a new Camera object
         *
         * @param projection_matrix - Camera projection matrix P = K[R|t]
         * @param distortion_coeffs
         * @param size_mult - multiplication factor to scale camera img
         */
        Camera(const Eigen::Matrix<double, 3, 4> &projection_matrix, std::vector<double>& distortion_coeffs, cv::Size size_of_img,  float size_mult)
        {
            this->P_matrix = projection_matrix; 
            img_size = size_of_img; 
            
            Eigen::Matrix3d rotation_matrix;
            Eigen::Matrix3d smallP = projection_matrix.block<3,3>(0,0); // M
            Eigen::Matrix3d Q;
            Q << 0, 0, 1.0,
                 0, 1.0, 0,
                 1.0, 0, 0;

            Eigen::MatrixXd O, C, B, A;
            O = Q * smallP * smallP.transpose() * Q;
            C = O.llt().matrixL();
            B = Q * C * Q;

            this->K = B / B(2,2);

            A = K.inverse() * smallP;
            double l = std::pow((1 / A.determinant()), 1/3);
            rotation_matrix = l * A;

            this->fx = this->K(0, 0);
            this->fy = this->K(1, 1);
            this->cx = this->K(0, 2);
            this->cy = this->K(1, 2);

            t = projection_matrix.col(3);
            t = K.inverse()*t;

            baseline = t.norm(); //normalize translation vector to get distance from coordinate frame center
            pose = Sophus::SE3d(rotation_matrix, t); //get Matrix that describes position of camera on e.g robot, car
            pose_inv = pose.inverse();

            this->dist_coeffs = distortion_coeffs;

            std::cout << "original K = " << K << "\n"; 
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
        Camera(double fx, double fy, double cx, double cy, double baseline,const Sophus::SE3d &pose, std::vector<double>& distortion_coeffs,cv::Size size_of_img, float size_mult)
            : fx(fx), fy(fy), cx(cx), cy(cy), baseline(baseline), pose(pose), dist_coeffs(distortion_coeffs), img_size(size_of_img)
        {
            pose_inv = pose.inverse();
            K <<fx, 0, cx,
            0, fy, cy,
            0, 0, 1;

            K = K* size_mult;
        }

        /**
         * @brief get undistorted (rectified) img
         * @param[in/out] img - source img
         */
        void imgRectification(cv::Mat &img, cv::Mat &corrected_img);

        Sophus::SE3d getPose() const {return pose; } // parameters getters
        Eigen::Matrix3d getK() const {return K; }
        Eigen::Matrix<double, 3, 4> getPmatrix() const {return P_matrix; } 

        cv::Mat getK_cv() { 
            cv::Mat K_cv;
            cv::eigen2cv(K, K_cv);
            return  K_cv; 
        }

        void insertNewProjectionMatrix(const Eigen::Matrix<double, 3, 4>& new_P_matrix); 

        // coordinate transform: world, camera, pixel
        Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw);
        Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_cw);
        Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);
        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);
        Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_cw, double depth = 1);
        Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw);

    };

    class StereoCameraSet
    {
    public: 
        StereoCameraSet(std::shared_ptr<Camera> left_camera, std::shared_ptr<Camera> right_camera )
        {
            //* get intrinsice matricies 
            cv::Mat left_K = left_camera->getK_cv(); 
            cv::Mat right_K = right_camera->getK_cv(); 
            left_K.convertTo(left_K, CV_64F);
            right_K.convertTo(right_K, CV_64F);

            //* Get transformation matricies and calculate transformation betweenen cameras 
            Eigen::Matrix4d left_Rt, right_Rt; 


            left_Rt = left_camera->getPose().matrix(); 
            right_Rt = right_camera->getPose().matrix(); 

            T_c1_c2 = right_Rt.inverse() * left_Rt; // valid for euroc dataset, not sure if for others too  
            R_c1_c2 = T_c1_c2.block<3,3>(0,0);
            t_c1_c2 = T_c1_c2.col(3).head<3>() / T_c1_c2.col(3).coeff(3); 

            cv::eigen2cv(R_c1_c2, R_c1_c2_mat); 
            cv::eigen2cv(t_c1_c2, t_c1_c2_mat); 
            R_c1_c2_mat.convertTo(R_c1_c2_mat, CV_64F);
            t_c1_c2_mat.convertTo(t_c1_c2_mat, CV_64F);

            cv::Size img_size = left_camera->img_size; 
            auto left_dist_coeffs = left_camera->dist_coeffs; 
            auto right_dist_coeffs = right_camera->dist_coeffs; 

            cv::stereoRectify(left_K, left_dist_coeffs, right_K, right_dist_coeffs, img_size, R_c1_c2_mat, t_c1_c2_mat, R1, R2, new_left_P, new_right_P, Q, 0, -1); 

            cv::initUndistortRectifyMap(left_K,left_dist_coeffs, R1, new_left_P, img_size, CV_16SC2, map11, map12); 
            cv::initUndistortRectifyMap(right_K,right_dist_coeffs, R2, new_right_P, img_size, CV_16SC2, map21, map22); 

            Eigen::Matrix<double,3,4> new_left_projection_matrix, new_right_projection_matrix; 
            cv::cv2eigen(new_left_P, new_left_projection_matrix); 
            cv::cv2eigen(new_right_P, new_right_projection_matrix);

            std::cout << "R1 = \n" << R1 << "\n";  
            std::cout << "new P1 = \n" << new_left_P << "\n"; 
            std::cout << "new P2 = \n" << new_right_P << "\n";  

            left_camera->insertNewProjectionMatrix(new_left_projection_matrix); 
            right_camera->insertNewProjectionMatrix(new_right_projection_matrix); 

        }

        void rectifyStereoImgs(cv::Mat& left_img, cv::Mat& right_img)
        {
            // std::cout << "Map11 = " << map11 << "\n"; 
            cv::remap(left_img, left_img, map11, map12, cv::INTER_LINEAR); 
            cv::remap(right_img, right_img, map21, map22, cv::INTER_LINEAR); 
        }

    private: 
        
        Eigen::Matrix4d T_c1_c2; 
        Eigen::Matrix3d R_c1_c2; 
        Eigen::Vector3d t_c1_c2; 
        cv::Mat R_c1_c2_mat, t_c1_c2_mat; 

        cv::Mat new_left_P, new_right_P, R1, R2, map11, map12, map21, map22, Q; 


    }; 

} //! end of namespace
