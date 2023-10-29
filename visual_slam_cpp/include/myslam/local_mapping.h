#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/tools.hpp"

namespace myslam {
    
    class Map;
    class LocalMapping {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        LocalMapping();
        void setLocalMapping(std::shared_ptr<Map> map_ptr, std::shared_ptr<Camera> left, std::shared_ptr<Camera> right)
        {
            map = map_ptr; 
            cam_left_ = left;
            cam_right_ = right;
        }
        void UpdateMap();
        void Stop();

    private:
        void LocalMappingThread();
        void LocalBundleAdjustment(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        std::shared_ptr<Camera> cam_left_ = nullptr, cam_right_ = nullptr;
    };

}  // namespace myslam
