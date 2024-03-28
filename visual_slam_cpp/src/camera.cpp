/**
 * @file camera.cpp
 * @author mrostocki 
 * @brief camera pinhole model - declaration of member functions
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "mrVSLAM/camera.hpp" 

namespace mrVSLAM
{
    void Camera::imgRectification(cv::Mat &img)
    {
        //cv::undistort(); 
    }

    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw) {
        return pose * T_cw * p_w;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_cw) {
        return T_cw.inverse() * pose_inv * p_c;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c) {
        return Eigen::Vector2d( fx * p_c(0, 0) / p_c(2, 0) + cx,
                                fy * p_c(1, 0) / p_c(2, 0) + cy );
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth) 
    {
        return Eigen::Vector3d((p_p(0, 0) - cx) * depth / fx,
                               (p_p(1, 0) - cy) * depth / fy,
                               depth);
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_cw) 
    {
        return camera2pixel(world2camera(p_w, T_cw));
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_cw, double depth) 
    {   
        Eigen::Vector3d p2c = pixel2camera(p_p, depth); 
        return camera2world(p2c, T_cw);
    }

} //! end of namespace 