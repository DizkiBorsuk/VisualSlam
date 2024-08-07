/**
 * @file slam.hpp
 * @author mrostocki
 * @brief
 * @version 0.1
 * @date 2024-03-09
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "mrVSLAM/common_includes.hpp"
#include "mrVSLAM/camera.hpp"
#include "mrVSLAM/local_mapping.hpp"
#include "mrVSLAM/loop_closing.hpp"
#include "mrVSLAM/stereo_tracking.hpp"
#include "mrVSLAM/mono_tracking.hpp"
#include "mrVSLAM/read_dataset.hpp"
#include "mrVSLAM/visualizer.hpp"
#include "mrVSLAM/map.hpp"

namespace mrVSLAM
{
    /**
     * @brief main visual slam algorithm class
     * @details
     */
    class SLAM
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Construct a new SLAM object
         * @details !NOT IMPLEMENTED YET! construct Visual SLAM algorithm object using configuration file
         * @param config_path path to configuration file that contains all parameters
         */
        SLAM(std::string config_path);
        /**
         * @brief Construct a new SLAM object
         * @details construct Visual SLAM algorithm object by passing basic parameters
         * @param dataset_to_run
         * @param path_to_dataset
         * @param type_of_algorithm
         * @param loop_closer
         */
        SLAM(std::string path_to_dataset,DatasetVersion dataset_to_run, SLAM_TYPE type_of_algorithm, bool loop_closer = false);

        /**
         * @brief Set the Slam Parameters object
         *
         * @param type_of_detector
         * @param num_of_tracked_points
         * @param min_tracking_points
         * @param resize
         * @param show_cam_img
         * @param vocab_path
         */
        void setSlamParameters(DetectorType type_of_detector = DetectorType::GFTT,
                               unsigned int num_of_tracked_points = 150,
                               unsigned int min_tracking_points = 80,
                               float resize = 1.0f,
                               bool show_cam_img = true,
                               std::string vocab_path = "./dictionaries/orbvoc.dbow3");

        /**
         * @brief initialize SLAM, read dataset and start all the modules
         */
        void initSLAM();

        /**
         * @brief algorithm main loop
         */
        void runSLAM();

        /**
         * @brief results generation and saving
         *
         * @param plot true/false plot results
         */
        void outputSlamResult(const bool plot = true);


    private:
        /**
         * @brief Create a New Frame and add it to tracking, runs
         *
         * @return true
         * @return false
         */
        bool createNewFrameAndTrack(const std::string left_img_path, const std::string right_img_path);

        void saveResultsToCSV(const ResultStruct &results);
        void saveTrajectoryToNetCDF(std::vector<Eigen::Matrix<double, 3,4>>& trajectory,
                                    std::vector<Eigen::Matrix<double, 3,4>>& kf_trajectory,
                                    std::vector<Eigen::Matrix<double, 3,4>>& no_kf_trajectory,
                                    std::vector<Eigen::Matrix<double, 3,4,1>>& gf_kf_trajectory);

    private:
        bool vslam_failed = false;
        bool img_distorted = false; 

        // constructor parameters
        std::string dataset_path;
        bool use_loop_closing = false;
        SLAM_TYPE tracking_type = SLAM_TYPE::STEREO;
        DatasetVersion dataset_type = DatasetVersion::KITTI;
        //rest of the parameters
        DetectorType detector_type = DetectorType::GFTT;
        int number_of_points = 150;
        int min_tracking_points = 80;
        float img_size_opt = 1.0f;
        bool show_cam_img = true;
        std::string vocab_path = "./dictionaries/orbvoc.dbow3";

        //ptrs to variosu modules
        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<LoopCloser> loop_closer= nullptr;
        std::shared_ptr<Camera> left_camera = nullptr;
        std::shared_ptr<Camera> right_camera = nullptr;
        std::shared_ptr<StereoTracking> stereo_tracking = nullptr;
        std::shared_ptr<MonoTracking> mono_tracking = nullptr;
        std::shared_ptr<Visualizer> visualizer = nullptr;
        std::unique_ptr<Dataset> dataset = nullptr;
        std::unique_ptr<StereoCameraSet> stereo_set = nullptr; 

        // result data
        std::vector<float> loop_times;
        std::vector<std::shared_ptr<Frame>> all_frames;
    };

} //! end of namespace
