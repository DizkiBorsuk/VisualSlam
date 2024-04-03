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
     * @class SLAM 
     * @brief main visual slam algorithm class 
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
         * @param path_to_dataset 
         * @param type_of_algorithm 
         * @param loop_closer 
         */
        SLAM(std::string path_to_dataset, SLAM_TYPE type_of_algorithm, bool loop_closer = false);

        /**
         * @brief Set the Slam Parameters object
         * @details if not using config file and want to set more parameters use this function 
         * @param num_of_tracked_points number of detected points by detector/extractor 
         * @param type_of_detector 
         * @param resize 
         * @param show_cam_img 
         * @param vocab_path path to BoW vocabulary, needed only for loop closing  
         */
        void setSlamParameters(DetectorType type_of_detector = DetectorType::GFTT, unsigned int num_of_tracked_points = 150, float resize = 1.0f, 
                               bool show_cam_img = true, std::string vocab_path = "./dictionaries/orbvoc.dbow3"); 

        /**
         * @brief initialize SLAM, read dataset and start all the modules 
         */
        void initSLAM(); 

        /**
         * @brief algorithm main loop 
         */
        void runSLAM(); 
        /**
         * @brief Create a New Frame And Track object
         * 
         * @return true 
         * @return false 
         */
        bool createNewFrameAndTrack(); 
        void outputSlamResult(); 


    private: 
        void saveResults(); 
        void saveTrajectoryAndMap(); 

    private: 
        // constructor parameters 
        std::string dataset_path; 
        bool use_loop_closing = false; 
        SLAM_TYPE tracking_type = SLAM_TYPE::STEREO; 
        //rest of the parameters 
        DetectorType detector_type = DetectorType::GFTT; 
        int number_of_points = 150; 
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
        std::shared_ptr<KITTI_Dataset> dataset = nullptr; 
        
        // result data
        std::vector<float> loop_times; 
        std::vector<Eigen::Matrix<double, 3,4>> trajectory;  
        std::vector<std::shared_ptr<Frame>> all_frames; 
        ResultStruct results; 
    }; 

} //! end of namespace 