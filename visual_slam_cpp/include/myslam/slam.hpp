#pragma once
#include "myslam/local_mapping.hpp"
#include "myslam/common_include.hpp"
#include "myslam/read_dataset.hpp"
#include "myslam/stereo_tracking.hpp"
#include "myslam/visualizer.hpp"
#include "myslam/loop_closing.hpp"

namespace myslam {

    enum class slamType {stereo_opf, stereo_matching, mono}; 

    class StereoTracking_Match; 
    class MonoTracking; 
    
    class SLAM {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// constructor with config file
        SLAM(std::string &config_path, slamType type_of_algorithm, bool loop_closer = false, float resize = 1.0);

        void Init();
        void Run();
        bool createNewFrameAndTrack();
        void output(); 


        std::vector<int> performance; 
        std::vector<Eigen::Matrix<double, 3,4>> trajectory;  

    private:
        std::string dataset_path;
        slamType algorithm_type = slamType::stereo_opf; 

        bool use_matching = false; 
        bool use_loop_closing = false; 
        float img_size_opt = 1.0; 

        int current_image_index_ = 0;

        std::shared_ptr<StereoTracking_OPF> stereoTracking = nullptr;
        std::shared_ptr<StereoTracking_Match> stereoTracking_with_match = nullptr; 
        std::shared_ptr<MonoTracking> monoTracking = nullptr; 

        std::shared_ptr<LocalMapping> local_mapping = nullptr;
        std::shared_ptr<Map> map = nullptr;
        std::shared_ptr<Visualizer> visualizer = nullptr;
        std::shared_ptr<LoopClosing> loop_closer = nullptr; 

        // dataset
        std::shared_ptr<KITTI_Dataset> dataset = nullptr;
        std::shared_ptr<DBoW3::Vocabulary> vocab = nullptr; 

        std::shared_ptr<Camera> left_camera = nullptr; 
        std::shared_ptr<Camera> right_camera = nullptr; 

        std::string vocab_path = "orbvoc.dbow3"; 

    };
}  
