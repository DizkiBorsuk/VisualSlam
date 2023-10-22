#pragma once
#include "myslam/local_mapping.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/stereo_tracking.hpp"
#include "myslam/viewer.h"

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

        std::shared_ptr<StereoTracking> stereoTracking = nullptr;
        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<Visualizer> visualizer = nullptr;

        // dataset
        std::shared_ptr<Dataset> dataset_ = nullptr;

    };
}  
