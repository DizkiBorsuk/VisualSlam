#include "myslam/common_include.hpp"
#include "DBoW3/DBoW3.h"

namespace myslam
{
    class Map; 
    class Frame; 
    class Camera; 
    class StereoTracking_OPF; 

    class LoopClosing
    {
    public: 
        LoopClosing(); 
        
        void setLoopCloser(std::shared_ptr<Map> map_ptr, std::shared_ptr<StereoTracking_OPF> tracking_ptr)
        {

        }

        void runLoopCloser(); 

        void addNewKeyframe(std::shared_ptr<Frame> new_keyframe)
        {
            std::unique_lock<std::mutex> lock(loop_closer_mutex); 
            current_frame = new_keyframe; 
        }

    private: 
        std::shared_ptr<Frame> current_frame = nullptr; 
        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<StereoTracking_OPF> tracking = nullptr;  

        std::thread loop_closer_thread; 
        std::mutex loop_closer_mutex; 

        DBoW3::Vocabulary vocabulary; 
        DBoW3::Database database; 

    }; 
}