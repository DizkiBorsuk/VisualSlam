#include "myslam/common_include.hpp"
#include "DBoW3/DBoW3.h"

namespace myslam
{
    class Map; 
    class Frame; 
    class Camera; 
    class StereoTracking_OPF; 
    class LocalMapping; 

    class LoopClosing
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LoopClosing(std::shared_ptr<DBoW3::Vocabulary> vocab); 
        void runLoopCloser(); 
        void globalBundleAdjustment(unsigned int loop_candidate_1_id, unsigned int loop_candidate_2_id); 
        void end(); 

        void setLoopCloser(std::shared_ptr<Map> map_ptr, 
                           std::shared_ptr<DBoW3::Vocabulary> vocab_ptr, 
                           std::shared_ptr<LocalMapping> l_map_ptr,
                           std::shared_ptr<Camera> l_cam_ptr, 
                           std::shared_ptr<Camera> r_cam_ptr)
        {
            map = map_ptr; 
            vocabulary = vocab_ptr; 
            local_mapping = l_map_ptr; 
            camera_left = l_cam_ptr; 
            camera_right = r_cam_ptr; 
        }

        void addCurrentKeyframe(std::shared_ptr<Frame> new_keyframe); 

        std::vector<std::array<std::shared_ptr<Frame>, 2>> keyframe_pairs; 
    private: 
        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<LocalMapping> local_mapping = nullptr;  
        std::shared_ptr<DBoW3::Vocabulary> vocabulary = nullptr; 

        std::shared_ptr<Camera> camera_left = nullptr; 
        std::shared_ptr<Camera> camera_right = nullptr; 

        std::thread loop_closer_thread; 
        std::mutex loop_closer_mutex; 

        DBoW3::Database database; 

        std::condition_variable map_update;
        std::atomic<bool> loop_closer_running;

    }; 
}