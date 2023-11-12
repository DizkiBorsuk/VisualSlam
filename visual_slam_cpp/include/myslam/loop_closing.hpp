#include "myslam/common_include.hpp"

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
        
        setLoopCloser(std::shared_ptr<Map> map_ptr, std::shared_ptr<StereoTracking_OPF> tracking_ptr)
        {

        }
    private: 

        std::shared_ptr<Map> map = nullptr; 
        std::shared_ptr<StereoTracking_OPF> tracking = nullptr;  

    }; 
}