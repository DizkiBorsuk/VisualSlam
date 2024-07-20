/**
 * @file read_dataset.hpp
 * @author your name (you@domain.com)
 * @brief reading kitti dataset stuff
 * @version 0.1
 * @date 2024-03-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "mrVSLAM/common_includes.hpp"

namespace mrVSLAM
{
    /**
     * @brief class for reading in kitti dataset https://www.cvlibs.net/datasets/kitti/
     *
     */

    class Dataset
    {
    public:
        virtual ~Dataset()  = default;

        virtual void readCalibData() = 0;
        virtual void readGTposes() = 0;
        virtual void showPmatricies()= 0;
        virtual int getCurrentSequence() = 0;
        virtual std::vector<double> returnLeftCamDistCoeffs() = 0;
        virtual std::vector<double> returnRightCamDistCoeffs() = 0; 
        virtual cv::Size returnDatasetImgSize() = 0; 

        virtual std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> retrunGTposes() = 0;
        virtual Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP0() = 0;
        virtual Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP1() = 0;

    };


    class KITTI_Dataset : public Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; //Projection matrix of left grayscale camera
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1; //Projection matrix of right grayscale camera
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P2; //Projection matrix of left rgb camera - unused, I don't need rgb images
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P3;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses; //poses are reprensented as a 3x4 transformation matrix:  3x3 - rotation matrix + 3x1 translation vector

        KITTI_Dataset(const std::string dataset_path);
        ~KITTI_Dataset() override = default;

        /**
         * @brief Calib.txt contains camera calibration data -> Projection matricies (3x4),
         *  projection matrix contains intrinsic (focal lengths, camera center) and extrinsic parameters.
         *  Kitti dataset stores projection matricies in flat shape, that is as a array with 12 elemets,
         *  each row is diffrent matrix
         */
        void readCalibData() override; //get camera projection matrixies from calibration file
        void showPmatricies() override;
        void readGTposes() override; //get set of ground truth poses
        int getCurrentSequence() override;

        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> retrunGTposes() override;
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP0() override;
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP1() override;
        cv::Size returnDatasetImgSize() override; 

        std::vector<double> returnLeftCamDistCoeffs() override;
        std::vector<double> returnRightCamDistCoeffs() override;

    private:
        std::string path_to_dataset;
        std::string camera_calibration_path;
        std::string gt_poses_path;
        cv::Size dataset_img_size = cv::Size(1241, 376); 
    };


    class EuRoC_Dataset : public Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P0; //Projection matrix of left grayscale camera
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> P1; //Projection matrix of right grayscale camera
        std::vector<double> left_distortion_coeffs, right_distortion_coeffs;

        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> ground_truth_poses;

        EuRoC_Dataset(const std::string dataset_path);
        ~EuRoC_Dataset() override = default;

        void readCalibData() override; //get camera projection matrixies from calibration file
        void showPmatricies() override;
        void readGTposes() override; //get set of ground truth poses
        int getCurrentSequence() override;
        std::vector<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> retrunGTposes() override;
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP0() override;
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> returnP1() override;
        cv::Size returnDatasetImgSize() override; 

        std::vector<double> returnLeftCamDistCoeffs() override;
        std::vector<double> returnRightCamDistCoeffs() override;

    private:
        std::string path_to_dataset;
        std::string left_camera_calibration_path;
        std::string right_camera_calibration_path;
        std::string gt_poses_path;

        std::vector<double> left_cam_dist_coeffs; 
        std::vector<double> right_cam_dist_coeffs;

        cv::Size dataset_img_size = cv::Size(752, 480); 
    };

} //! end of namespace
