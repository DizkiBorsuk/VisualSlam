#include "myslam/common_include.hpp"
#include "myslam/frame.hpp"
#include "myslam/map.hpp"
#include "myslam/tools.hpp"

namespace myslam {
    
    class Map;
    class LocalMapping {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
        std::thread local_mapping_thread;
        std::mutex local_mapping_mutex;

        std::condition_variable map_update_;
        std::atomic<bool> local_mapping_running;

        std::shared_ptr<Camera> cam_left_ = nullptr, cam_right_ = nullptr;
    };

}  // namespace myslam
