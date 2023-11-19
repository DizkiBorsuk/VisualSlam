#pragma once
#include "myslam/local_mapping.hpp"
#include "myslam/common_include.hpp"
#include "myslam/read_dataset.hpp"
#include "myslam/stereo_tracking.hpp"
#include "myslam/visualizer.hpp"
#include "DBoW3/DBoW3.h"

namespace myslam {

    
    class StereoSLAM {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /// constructor with config file
        StereoSLAM(std::string &config_path, float resize = 1.0);

        void Init();
        void Run();
        bool createNewFrameAndTrack();
        void output(); 


        std::vector<int> performance; 
        std::vector<Eigen::Matrix<double, 3,4>> trajectory;  

    private:
        std::string dataset_path;
        float img_size_opt = 1.0; 

        int current_image_index_ = 0;
        bool inited_ = false;

        std::shared_ptr<StereoTracking_OPF> stereoTracking = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<Visualizer> visualizer = nullptr;

        // dataset
        std::shared_ptr<KITTI_Dataset> dataset = nullptr;
        std::shared_ptr<DBoW3::Vocabulary> vocab = nullptr; 

        std::shared_ptr<Camera> left_camera = nullptr; 
        std::shared_ptr<Camera> right_camera = nullptr; 

        std::string vocab_path = "orbvoc.dbow3"; 

    };
}  
