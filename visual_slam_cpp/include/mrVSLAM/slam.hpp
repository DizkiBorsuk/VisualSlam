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
        
        SLAM(std::string config_path); 
        SLAM(std::string path_to_dataset, SLAM_TYPE type_of_algorithm, bool loop_closer = false);

        void initSLAM(); 
        void runSLAM(); 
        bool createNewFrameAndTrack(); 
        void outputSlamResult(); 
        void setSlamParameters(unsigned int num_of_tracked_points, DetectorType type_of_detector, float resize = 1.0f); 

    private: 
        void saveResults(); 
        void saveTrajectoryAndMap(); 

    private: 

        bool use_loop_closing = false; 
        SLAM_TYPE tracking_type = SLAM_TYPE::STEREO; 
        DetectorType detector_type = DetectorType::GFTT; 
        int number_of_points = 500; 
        float img_size_opt = 1.0f; 

        std::string dataset_path; 
        std::string vocab_path = "./dictionaries/orbvoc.dbow3"; 

        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<LoopCloser> loop_closer= nullptr; 
        std::shared_ptr<Camera> left_camera = nullptr; 
        std::shared_ptr<Camera> right_camera = nullptr; 
        std::shared_ptr<StereoTracking> stereo_tracking = nullptr; 
        std::shared_ptr<MonoTracking> mono_tracking = nullptr; 
        std::shared_ptr<Visualizer> visualizer = nullptr; 
        std::shared_ptr<KITTI_Dataset> dataset = nullptr; 

        std::vector<bool> loop_times; 
        std::vector<Eigen::Matrix<double, 3,4>> trajectory;  
        std::vector<std::shared_ptr<Frame>> all_frames; 
        ResultStruct results; 
    }; 

} //! end of namespace 